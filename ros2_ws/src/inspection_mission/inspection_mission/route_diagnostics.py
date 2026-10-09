"""Simulation-only route acceptance measurements.

Ground truth is an acceptance ruler only and is never fed back to AMCL/Nav2.

Fault layers (do not collapse into one hard stop):
  - control: odom/TF dependency for navigation observation
  - acceptance: GT missing/stale → this trial cannot be accepted
  - warning: localization quality (covariance / AMCL↔GT) → observe only

Stop semantics match scripts/navigate_to_pose_gate.py:
  settle within 1.0 s, then no re-violation for the rest of the window.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

_AMCL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


class RouteDiagnostics:
    POS_GATE_M = 0.20
    YAW_GATE_DEG = 10.0
    AMCL_GATE_M = 0.10
    STOP_WINDOW_SEC = 1.0
    VEL_GATE = 0.02
    OMEGA_GATE = 0.05

    # Control dependency: navigation cannot be observed safely.
    ODOM_STALE_SEC = 1.0
    TF_STALE_SEC = 1.0
    # Acceptance ruler: simulation-only; missing ⇒ gate UNKNOWN/FAIL, not Nav abort.
    GT_STALE_SEC = 1.0
    # AMCL is motion-gated (update_min_d/a). Do not treat "no new pose while
    # stopped" as a control failure. Only flag extremely old with motion.
    AMCL_MOVING_STALE_SEC = 5.0
    TRANSFORM_TOLERANCE_SEC = 0.5

    def __init__(self, node, origin, output_dir: str = '') -> None:
        self.node = node
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.gt = None
        self.odom = None
        self.amcl = None
        self.gt_count = 0
        self.odom_count = 0
        self.amcl_count = 0
        self.gt_recv_mono = None
        self.odom_recv_mono = None
        self.amcl_recv_mono = None
        self.odom_samples = []
        self._stop_t0 = None
        self.health_log: list[dict] = []
        self.acceptance_blocked = False
        self.acceptance_block_reason = None
        # Transient TF extrapolation after spawn/reconfigure is common; only a
        # sustained outage is a control hard-stop.
        # Fresh TransformListener starts with an empty buffer — do NOT treat
        # pre-warmup failures as control_dependency:tf_unavailable (that false
        # abort killed A immediately after respawn in patrol_stress).
        self._tf_fail_since = None
        self._tf_ever_ok = False
        self._last_tf_info = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.node.create_subscription(
            Odometry, '/ground_truth', self._on_gt, qos_profile_sensor_data,
        )
        self.node.create_subscription(
            Odometry, '/odom', self._on_odom, qos_profile_sensor_data,
        )
        self.node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._on_amcl, _AMCL_QOS,
        )
        self.output_dir = Path(output_dir) if output_dir else None
        self.ready_state = None

    def _on_gt(self, msg: Odometry) -> None:
        self.gt = msg
        self.gt_count += 1
        self.gt_recv_mono = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        self.odom_count += 1
        self.odom_recv_mono = time.monotonic()
        if self._stop_t0 is None:
            return
        # Relative time from stop-window start (same as navigate_to_pose_gate).
        self.odom_samples.append((
            time.monotonic() - self._stop_t0,
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.angular.z),
        ))

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl = msg
        self.amcl_count += 1
        self.amcl_recv_mono = time.monotonic()

    def wait_ready(self) -> None:
        """Wait for GT/odom/AMCL plus at least one successful map→base TF.

        wait_for_nav_ready runs in another process with its own TF buffer.
        This node's listener must warm up before fault() monitors navigation.
        """
        deadline = time.monotonic() + 30.0
        topics_ready_at = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.gt is None or self.odom is None or self.amcl is None:
                continue
            if topics_ready_at is None:
                topics_ready_at = time.monotonic()
            tf_info = self._lookup_tf()
            if tf_info.get('ok'):
                self._tf_ever_ok = True
                self._tf_fail_since = None
                self.ready_state = self.snapshot()
                self.ready_state['tf_warmup'] = {
                    'ok': True,
                    'waited_sec': round(time.monotonic() - topics_ready_at, 3),
                    **{k: tf_info.get(k) for k in (
                        'stamp_sec', 'clock_sec', 'stamp_minus_clock_sec',
                    )},
                }
                return
            # Topics up but TF still filling — keep spinning until deadline.
        self.ready_state = self.snapshot()
        self.ready_state['tf_warmup'] = {
            'ok': False,
            'reason': (self._last_tf_info or {}).get('reason', 'tf_timeout'),
        }
        self.node.get_logger().warn(
            'PATROL_DIAG: topics ready but map→base_footprint TF not warmed; '
            'gates may be UNKNOWN / early tf warnings expected',
        )

    def snapshot(self) -> dict:
        result = {
            'ground_truth': None,
            'amcl': None,
            'covariance': None,
            'odom_speed': None,
            'topic_counts': {
                'ground_truth': self.gt_count,
                'odom': self.odom_count,
                'amcl_pose': self.amcl_count,
            },
        }
        if self.gt is not None:
            pose = self.gt.pose.pose
            result['ground_truth'] = {
                'x': round(float(pose.position.x), 4),
                'y': round(float(pose.position.y), 4),
                'yaw': round(yaw_from_quaternion(pose.orientation), 4),
            }
        if self.amcl is not None:
            pose = self.amcl.pose.pose
            result['amcl'] = {
                'x': round(float(pose.position.x), 4),
                'y': round(float(pose.position.y), 4),
                'yaw': round(yaw_from_quaternion(pose.orientation), 4),
            }
            result['covariance'] = [
                round(float(self.amcl.pose.covariance[index]), 4)
                for index in (0, 7, 35)
            ]
        if self.odom is not None:
            result['odom_speed'] = {
                'linear_x': round(float(self.odom.twist.twist.linear.x), 4),
                'angular_z': round(float(self.odom.twist.twist.angular.z), 4),
            }
        return result

    def _age(self, recv_mono) -> float | None:
        if recv_mono is None:
            return None
        return time.monotonic() - recv_mono

    def _lookup_tf(self) -> dict:
        info = {'ok': False, 'reason': 'lookup_failed', 'transient': False}
        try:
            # Time() == 0 → latest transform (do not pin to a wall/sim snapshot).
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(),
                timeout=Duration(seconds=0.05),
            )
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            clock = self.node.get_clock().now().nanoseconds * 1e-9
            delta = (stamp - clock) if clock > 0 else None
            # AMCL may stamp into the future within transform_tolerance.
            future_ok = delta is None or delta <= self.TRANSFORM_TOLERANCE_SEC + 0.05
            past_ok = delta is None or delta >= -self.TF_STALE_SEC
            info = {
                'ok': bool(future_ok and past_ok),
                'transient': False,
                'stamp_sec': round(stamp, 3),
                'clock_sec': round(clock, 3) if clock > 0 else None,
                'stamp_minus_clock_sec': round(delta, 3) if delta is not None else None,
                'within_transform_tolerance': future_ok,
            }
            if not info['ok']:
                info['reason'] = (
                    'tf_stamp_too_far_future' if not future_ok else 'tf_stamp_too_old'
                )
                # Stamp skew right after reconfigure is usually transient.
                info['transient'] = True
        except TransformException as exc:
            text = str(exc)
            transient = (
                'extrapolation into the past' in text
                or 'extrapolation into the future' in text
                or 'earliest data' in text
                or 'latest data' in text
                or 'not part of the same tree' in text
                or 'unconnected trees' in text
            )
            info = {'ok': False, 'reason': text, 'transient': transient}
        return info

    def classify_health(self) -> dict:
        """Layered health record. Does not by itself stop navigation."""
        now_speed = (0.0, 0.0)
        if self.odom is not None:
            now_speed = (
                abs(float(self.odom.twist.twist.linear.x)),
                abs(float(self.odom.twist.twist.angular.z)),
            )
        moving = now_speed[0] >= self.VEL_GATE or now_speed[1] >= self.OMEGA_GATE
        gt_age = self._age(self.gt_recv_mono)
        odom_age = self._age(self.odom_recv_mono)
        amcl_age = self._age(self.amcl_recv_mono)
        tf_info = self._lookup_tf()
        self._last_tf_info = tf_info
        if tf_info.get('ok'):
            self._tf_ever_ok = True
            self._tf_fail_since = None
        elif self._tf_ever_ok:
            # Only start the hard-stop clock after the buffer has worked once.
            self._tf_fail_since = self._tf_fail_since or time.monotonic()
        else:
            self._tf_fail_since = None
        tf_fail_age = (
            None if self._tf_fail_since is None
            else time.monotonic() - self._tf_fail_since
        )
        tf_hard = (
            self._tf_ever_ok
            and not tf_info.get('ok')
            and tf_fail_age is not None
            and tf_fail_age >= self.TF_STALE_SEC
        )

        issues = []
        # --- control ---
        if odom_age is None or odom_age > self.ODOM_STALE_SEC:
            issues.append({
                'level': 'control',
                'code': 'odom_stale',
                'odom_age_sec': odom_age,
            })
        if tf_hard:
            issues.append({
                'level': 'control',
                'code': 'tf_unavailable',
                'tf': tf_info,
                'tf_fail_age_sec': round(tf_fail_age, 3),
            })
        elif not tf_info.get('ok'):
            # Single-frame / brief extrapolation → warning only.
            issues.append({
                'level': 'warning',
                'code': 'tf_transient',
                'tf': tf_info,
                'tf_fail_age_sec': None if tf_fail_age is None else round(tf_fail_age, 3),
            })

        # --- acceptance (simulation ruler) ---
        if gt_age is None or gt_age > self.GT_STALE_SEC:
            issues.append({
                'level': 'acceptance',
                'code': 'ground_truth_stale',
                'gt_age_sec': gt_age,
            })

        # --- warning / conditional AMCL ---
        if self.amcl is None:
            issues.append({
                'level': 'warning',
                'code': 'amcl_missing',
            })
        elif moving and (amcl_age is None or amcl_age > self.AMCL_MOVING_STALE_SEC):
            # Only when the robot is moving should missing AMCL updates worry us.
            issues.append({
                'level': 'warning',
                'code': 'amcl_stale_while_moving',
                'amcl_age_sec': amcl_age,
            })
        # Stationary + no new AMCL is normal under update_min_d/a — not an issue.

        if self.amcl is not None:
            cov = [float(self.amcl.pose.covariance[i]) for i in (0, 7, 35)]
            if any(not math.isfinite(v) or v < 0 for v in cov):
                issues.append({'level': 'warning', 'code': 'covariance_invalid', 'covariance': cov})
            elif cov[0] > 0.25 or cov[1] > 0.25 or cov[2] > 0.20:
                issues.append({
                    'level': 'warning',
                    'code': 'covariance_high',
                    'covariance': [round(v, 4) for v in cov],
                })

        level = None
        for candidate in ('control', 'acceptance', 'warning'):
            if any(item['level'] == candidate for item in issues):
                level = candidate
                break
        return {
            'level': level,
            'issues': issues,
            'tf': tf_info,
            'ages': {
                'ground_truth': gt_age,
                'odom': odom_age,
                'amcl_pose': amcl_age,
            },
            'moving': moving,
            'topic_counts': {
                'ground_truth': self.gt_count,
                'odom': self.odom_count,
                'amcl_pose': self.amcl_count,
            },
        }

    def fault(self) -> str | None:
        """Hard-stop reason for control dependency only.

        Acceptance gaps and localization warnings are recorded via
        classify_health() / health_log and must not abort the Nav2 action.
        """
        health = self.classify_health()
        if health['level'] == 'acceptance':
            self.acceptance_blocked = True
            codes = [i['code'] for i in health['issues'] if i['level'] == 'acceptance']
            self.acceptance_block_reason = ','.join(codes) or 'acceptance_data_gap'
            self.health_log.append(health)
            return None
        if health['level'] == 'warning':
            # Throttle identical warnings in the log.
            if not self.health_log or self.health_log[-1].get('issues') != health['issues']:
                self.health_log.append(health)
            return None
        if health['level'] == 'control':
            self.health_log.append(health)
            codes = [i['code'] for i in health['issues'] if i['level'] == 'control']
            return f"control_dependency:{','.join(codes)}"
        return None

    def begin_stop_window(self) -> None:
        self.odom_samples.clear()
        self._stop_t0 = time.monotonic()

    def _analyze_stopped_window(self) -> dict:
        """Align with navigate_to_pose_gate._analyze_stopped_window."""
        detail = {
            'stopped': False,
            'settle_sec': None,
            're_violation': False,
            'max_v': 0.0,
            'max_w': 0.0,
            'sample_n': len(self.odom_samples),
        }
        if not self.odom_samples:
            detail['reason'] = 'no_odom_samples'
            return detail

        for t_sec, v, w in self.odom_samples:
            detail['max_v'] = max(detail['max_v'], abs(v))
            detail['max_w'] = max(detail['max_w'], abs(w))

        settle_sec = None
        for t_sec, v, w in self.odom_samples:
            if abs(v) < self.VEL_GATE and abs(w) < self.OMEGA_GATE:
                settle_sec = t_sec
                break
        detail['settle_sec'] = None if settle_sec is None else round(settle_sec, 3)
        if settle_sec is None:
            detail['reason'] = 'never_settled'
            return detail

        re_violation = any(
            t_sec > settle_sec
            and (abs(v) >= self.VEL_GATE or abs(w) >= self.OMEGA_GATE)
            for t_sec, v, w in self.odom_samples
        )
        detail['re_violation'] = re_violation
        detail['stopped'] = (
            settle_sec <= self.STOP_WINDOW_SEC and not re_violation
        )
        if not detail['stopped'] and settle_sec > self.STOP_WINDOW_SEC:
            detail['reason'] = 'settle_after_window'
        elif re_violation:
            detail['reason'] = 're_violation_after_settle'
        return detail

    def station_gate(self, target) -> dict:
        # Collect a clean one-second stop window after action success.
        # Require GT/odom to keep streaming in the window; do NOT require new AMCL
        # (stationary AMCL may stay quiet under update_min_d/a).
        gt_start = self.gt_count
        odom_start = self.odom_count
        stale_hits = 0
        deadline = time.monotonic() + self.STOP_WINDOW_SEC
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            gt_age = self._age(self.gt_recv_mono)
            odom_age = self._age(self.odom_recv_mono)
            if (
                gt_age is None or gt_age > self.GT_STALE_SEC
                or odom_age is None or odom_age > self.ODOM_STALE_SEC
            ):
                stale_hits += 1
                self.acceptance_blocked = True
                self.acceptance_block_reason = 'station_window_stale_data'

        stop_detail = self._analyze_stopped_window()
        self._stop_t0 = None
        gt_delta = self.gt_count - gt_start
        odom_delta = self.odom_count - odom_start
        coverage = {
            'gt_updates': gt_delta,
            'odom_updates': odom_delta,
            'stale_hits': stale_hits,
            # Heuristic: expect multiple samples in a 1s window at ~20–50 Hz.
            'gt_ok': gt_delta >= 5 and stale_hits == 0,
            'odom_ok': odom_delta >= 5 and stale_hits == 0,
        }

        result = {
            'pass': False,
            'position_error_m': None,
            'yaw_error_deg': None,
            'amcl_error_m': None,
            'covariance': None,
            'stopped': bool(stop_detail['stopped']),
            'stop_detail': stop_detail,
            'sample_coverage': coverage,
        }
        if self.acceptance_blocked:
            result['reason'] = self.acceptance_block_reason or 'acceptance_data_gap'
            return result
        if not coverage['gt_ok'] or not coverage['odom_ok']:
            result['reason'] = 'station_window_insufficient_samples'
            return result
        if self.gt is None:
            result['reason'] = 'ground_truth_missing'
            return result

        gt_pose = self.gt.pose.pose
        gt_x = float(gt_pose.position.x)
        gt_y = float(gt_pose.position.y)
        gt_yaw = yaw_from_quaternion(gt_pose.orientation)
        target_x, target_y, target_yaw = (float(value) for value in target)
        pos_error = math.hypot(gt_x - target_x, gt_y - target_y)
        yaw_error_deg = math.degrees(angle_error(gt_yaw, target_yaw))
        result.update(
            position_error_m=round(pos_error, 4),
            yaw_error_deg=round(yaw_error_deg, 2),
        )

        if self.amcl is not None:
            amcl_pose = self.amcl.pose.pose
            amcl_x = float(amcl_pose.position.x) + self.origin_x
            amcl_y = float(amcl_pose.position.y) + self.origin_y
            result['amcl_error_m'] = round(
                math.hypot(amcl_x - gt_x, amcl_y - gt_y), 4,
            )
            covariance = self.amcl.pose.covariance
            result['covariance'] = [
                round(float(covariance[index]), 4) for index in (0, 7, 35)
            ]

        result['pass'] = (
            pos_error <= self.POS_GATE_M
            and yaw_error_deg <= self.YAW_GATE_DEG
            and result['stopped']
            and result['amcl_error_m'] is not None
            and result['amcl_error_m'] <= self.AMCL_GATE_M
            and coverage['gt_ok']
            and coverage['odom_ok']
        )
        if not result['pass'] and 'reason' not in result:
            if not result['stopped']:
                result['reason'] = stop_detail.get('reason') or 'not_stopped'
            elif result['amcl_error_m'] is None:
                result['reason'] = 'amcl_missing'
            elif result['amcl_error_m'] > self.AMCL_GATE_M:
                result['reason'] = 'amcl_error'
            elif pos_error > self.POS_GATE_M:
                result['reason'] = 'position_error'
            elif yaw_error_deg > self.YAW_GATE_DEG:
                result['reason'] = 'yaw_error'
        return result

    def close(self) -> None:
        if self.output_dir:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            path = self.output_dir / 'health_log.json'
            path.write_text(json.dumps(self.health_log, indent=2) + '\n')
