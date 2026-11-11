#!/usr/bin/env python3
"""Mission Runtime: ExecuteInspection Action entry for MVP-R1..R7.

Action name: /inspection/execute
Routes:
  test_room_A   → station A only (R1–R6 compatible)
  test_room_ABC → A→B→C with B_to_C vias from patrol_route.yaml (R7)

Per station: nav → settle → new frame → same-frame YOLO+ROI.
Runtime sole SQLite writer: begin → persist each station → finalize.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from inspection_interfaces.action import ExecuteInspection
from inspection_mission.map_pose import pose_from_route_values
from inspection_mission.mission_archive import (
    MissionArchive,
    utc_now_iso,
)
from inspection_mission.mission_recovery_ipc import RecoveryRequestProcessor
from inspection_mission.mission_runner import MissionRunner
from inspection_mission.mission_state import (
    ALLOWED_ROUTE_ID,
    ALLOWED_ROUTE_IDS,
    AcceptCancelBridge,
    ErrorCode,
    ExecutionStatus,
    MissionActiveSlot,
    MissionRuntimeState,
    MissionTerminal,
    Phase,
    STATION_A,
)
from inspection_mission.nav_adapter import NavAdapter
from inspection_mission.robot_safety import (
    DEFAULT_ROBOT_ID,
    RobotInstanceGuard,
    default_runtime_dir,
)
from inspection_mission.route_plan import (
    aggregate_inspection_status,
    stations_for_route,
    transfer_through_world_poses,
)
from inspection_mission.station_capture import (
    CameraFrameBuffer,
    OdomSettleMonitor,
    PoseSnapshotter,
    capture_station_frame,
)
from inspection_mission.station_inspect import (
    DEFAULT_RULES,
    DEFAULT_WEIGHTS,
    inspect_station_image,
)


@dataclass
class StationWorkResult:
    ok: bool
    canceled: bool = False
    error_code: str = ErrorCode.NONE
    message: str = ''
    station_id: str = ''
    screenshot_path: str = ''
    image_stamp: str = ''
    frame_id: str = ''
    inspection_status: str = ''
    station_result_path: str = ''
    model_version: str = ''
    annotated_path: str = ''
    roi_rules_version: str = ''
    objects: Any = field(default_factory=dict)


class MissionRuntimeNode(Node):
    def __init__(self, *, nav_adapter: Optional[NavAdapter] = None) -> None:
        super().__init__(
            'mission_runtime_node',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        share = Path(get_package_share_directory('inspection_mission'))
        nav_share = Path(get_package_share_directory('navigation_config'))

        self.declare_parameter('route_file', str(share / 'config/patrol_route.yaml'))
        self.declare_parameter(
            'map_yaml', str(nav_share / 'maps/test_room.yaml'),
        )
        self.declare_parameter('action_name', '/inspection/execute')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('server_timeout_sec', 15.0)
        self.declare_parameter('send_timeout_sec', 15.0)
        self.declare_parameter('result_timeout_sec', 300.0)
        self.declare_parameter('cancel_timeout_sec', 10.0)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('settle_timeout_sec', 15.0)
        self.declare_parameter('settle_sec', 1.0)
        self.declare_parameter('frame_timeout_sec', 10.0)
        # Env defaults match scripts/mission_defaults.sh (Web/API/Runtime unify).
        _home_bags = Path.home() / 'inspection-robot' / 'bags'
        _default_capture = os.environ.get(
            'INSPECTION_CAPTURE_DIR',
            str(_home_bags / 'mission_captures'),
        )
        _default_sqlite = os.environ.get(
            'INSPECTION_ARCHIVE_DB',
            str(_home_bags / 'inspection_archive.sqlite3'),
        )
        self.declare_parameter('capture_output_dir', _default_capture)
        self.declare_parameter('yolo_weights', str(DEFAULT_WEIGHTS))
        self.declare_parameter('roi_rules', str(DEFAULT_RULES))
        self.declare_parameter('yolo_conf', 0.25)
        self.declare_parameter('sqlite_db', _default_sqlite)
        _default_runtime = os.environ.get(
            'INSPECTION_ROBOT_RUNTIME_DIR',
            str(default_runtime_dir()),
        )
        _default_robot = os.environ.get('INSPECTION_ROBOT_ID', DEFAULT_ROBOT_ID)
        self.declare_parameter('robot_runtime_dir', _default_runtime)
        self.declare_parameter('robot_id', _default_robot)

        route_file = Path(self.get_parameter('route_file').value)
        map_yaml = Path(self.get_parameter('map_yaml').value)
        self._route = yaml.safe_load(route_file.read_text(encoding='utf-8'))
        self._origin = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))['origin']
        for sid in ('A', 'B', 'C'):
            if sid not in self._route.get('stations', {}):
                raise ValueError(f'station {sid} missing in {route_file}')

        self._cb_group = ReentrantCallbackGroup()
        self.state = MissionRuntimeState()
        self.slot = MissionActiveSlot(lock=self.state.lock)
        self.accept_cancel = AcceptCancelBridge(self.state, self.slot)
        self._writer_enabled = False
        self._archive: Optional[MissionArchive] = None
        self._guard = RobotInstanceGuard(
            robot_id=str(self.get_parameter('robot_id').value),
            runtime_dir=Path(self.get_parameter('robot_runtime_dir').value),
        )
        # R1: flock BEFORE writable archive / accepting goals.
        flock_ok, flock_msg = self._guard.acquire_flock()
        if not flock_ok:
            self.state.set_safety_refuse(
                True, f'robot flock unavailable: {flock_msg}',
            )
            self.get_logger().error(
                f'Refusing goals — robot flock failed before archive open: {flock_msg}',
            )
        else:
            self._guard.write_instance_meta()
            self._archive = MissionArchive(
                Path(self.get_parameter('sqlite_db').value),
            )
            self._writer_enabled = True
            self.state.seed_used_mission_ids(self._archive.list_mission_ids())
            allow, reason, _snap = self._guard.startup_gate()
            # First-use only: missing safety file → bootstrap clear under flock.
            # Active/refuse/corrupt leftovers still require operator confirm.
            if not allow and reason == 'safety mark missing':
                boot_ok, boot_msg = self._guard.bootstrap_clear_if_missing()
                if boot_ok:
                    allow, reason, _snap = self._guard.startup_gate()
                    self.get_logger().info(
                        f'Bootstrapped clear safety mark on first use '
                        f'({boot_msg}); instance_id={self._guard.instance_id}',
                    )
                else:
                    self.get_logger().warn(
                        f'First-use bootstrap failed: {boot_msg}',
                    )
            if not allow:
                self.state.set_safety_refuse(True, reason)
                self.get_logger().warn(
                    f'Refusing goals until operator confirm: {reason} '
                    f'(instance_id={self._guard.instance_id})',
                )
            # R3: RUNNING shells refuse even if disk safety is clear.
            running = self._archive.list_running_mission_ids()
            if running:
                self.state.set_safety_refuse(
                    True,
                    f'RUNNING archive shells present: {running}; '
                    'operator confirm required',
                )
                self.get_logger().warn(
                    f'Refusing goals — RUNNING shells={running}',
                )
            elif allow:
                self.get_logger().info(
                    f'Safety gate clear robot_id={self._guard.robot_id} '
                    f'instance_id={self._guard.instance_id}',
                )
        self._nav = nav_adapter or NavAdapter(
            self,
            to_pose_action=str(self.get_parameter('nav_action_name').value),
        )
        self._camera = CameraFrameBuffer(
            self, topic=str(self.get_parameter('camera_topic').value),
        )
        self._settle = OdomSettleMonitor(
            self, topic=str(self.get_parameter('odom_topic').value),
        )
        self._poses = PoseSnapshotter(self)

        self._action_server = ActionServer(
            self,
            ExecuteInspection,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self._timeout_timer = self.create_timer(
            0.1, self._on_timeout_timer, callback_group=self._cb_group,
        )
        self._recovery = RecoveryRequestProcessor(
            guard=self._guard,
            handle_status=self._ipc_status,
            handle_arm=self._ipc_arm,
            handle_confirm=self._ipc_confirm,
        )
        self._recovery_timer = self.create_timer(
            0.2, self._on_recovery_timer, callback_group=self._cb_group,
        )
        n_seed = len(self.state.used_mission_ids)
        db_label = (
            str(self._archive.db_path) if self._archive is not None else '(no-writer)'
        )
        self.get_logger().info(
            f"Mission Runtime ready on {self.get_parameter('action_name').value} "
            f"(MVP-R7 multi-station; routes={sorted(ALLOWED_ROUTE_IDS)}; "
            f"sqlite={db_label}; seeded_missions={n_seed}; "
            f"robot={self._guard.robot_id}; instance={self._guard.instance_id}; "
            f"writer={self._writer_enabled})",
        )

    @property
    def active_runner(self):
        ctx = self.slot.active
        return None if ctx is None else ctx.runner

    def install_active_context(self, ctx) -> None:
        self.slot.install(ctx)

    def clear_active_context_if_owner(self, ctx) -> bool:
        return self.slot.clear_if_owner(ctx)

    def _goal_callback(self, goal_request: ExecuteInspection.Goal) -> GoalResponse:
        if not self._writer_enabled or self._archive is None:
            self.get_logger().warn(
                'Reject ExecuteInspection: archive writer disabled (flock fail)',
            )
            return GoalResponse.REJECT
        if self._guard.flock_failed or not self._guard.held:
            self.get_logger().warn('Reject ExecuteInspection: robot flock not held')
            return GoalResponse.REJECT
        # Atomic validate + reserve + ticket.open (V2-02B1).
        ok, code, message, _generation = self.state.reserve_goal(
            goal_request.mission_id,
            goal_request.route_id,
            self.accept_cancel.window,
        )
        if not ok:
            self.get_logger().warn(
                f'Reject ExecuteInspection: code={code} msg={message}',
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_recovery_timer(self) -> None:
        try:
            self._recovery.poll_once()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'recovery IPC poll failed: {exc}')

    def _handle_accepted_callback(self, goal_handle) -> None:
        mission_id = str(goal_handle.request.mission_id).strip()
        # Decide under lock; abort/execute only outside (V2-02B1 / §17.2).
        should_abort = False
        generation: Optional[int] = None
        with self.state.lock:
            generation = self.state.active_generation
            if (
                generation is None
                or not self.state.is_owner_unlocked(mission_id, generation)
            ):
                self.accept_cancel.window.clear_unlocked()
                should_abort = True
        if should_abort:
            self.get_logger().error(
                f'Accepted goal missing owner reservation mission_id={mission_id}',
            )
            goal_handle.abort()
            return
        assert generation is not None
        # consume ticket + slot.install in one critical section inside Bridge.
        self.accept_cancel.install_context_after_accepted(
            goal_handle, mission_id, generation,
        )
        goal_handle.execute()

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        # resolve_cancel: record under lock, request_cancel outside.
        if self.accept_cancel.resolve_cancel(goal_handle):
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def _on_timeout_timer(self) -> None:
        ctx = self.slot.active
        runner = None if ctx is None else ctx.runner
        if runner is not None:
            runner.poll_timeouts()

    def _station_pose(self, station_id: str):
        values = self._route['stations'][station_id]
        stamp = self.get_clock().now().to_msg()
        return pose_from_route_values(values, self._origin, stamp=stamp)

    def _via_pose(self, values):
        stamp = self.get_clock().now().to_msg()
        return pose_from_route_values(values, self._origin, stamp=stamp)

    def _publish_feedback(
        self, goal_handle, mission_id: str, station_id: str, phase: str, message: str,
    ) -> None:
        fb = ExecuteInspection.Feedback()
        fb.mission_id = mission_id
        fb.station_id = station_id
        fb.phase = phase
        fb.message = message
        goal_handle.publish_feedback(fb)

    def _fill_result(self, result: ExecuteInspection.Result, terminal: MissionTerminal) -> None:
        result.mission_id = terminal.mission_id
        result.execution_status = terminal.execution_status
        result.error_code = terminal.error_code
        result.message = terminal.message
        result.screenshot_path = terminal.screenshot_path or ''
        result.image_stamp = terminal.image_stamp or ''
        result.frame_id = terminal.frame_id or ''
        result.station_id = terminal.station_id or ''
        result.inspection_status = terminal.inspection_status or ''
        result.station_result_path = terminal.station_result_path or ''
        result.model_version = terminal.model_version or ''

    def _nav_goal_diag(self, station_id: str) -> dict:
        values = [float(v) for v in self._route['stations'][station_id]]
        ox, oy, oyaw = (float(v) for v in self._origin[:3])
        pose = self._station_pose(station_id)
        return {
            'station_id': station_id,
            'route_world': {
                'x': values[0],
                'y': values[1],
                'yaw': values[2],
                'frame': 'world',
            },
            'map_origin': {'x': ox, 'y': oy, 'yaw': oyaw},
            'nav_goal_map': {
                'x': float(pose.pose.position.x),
                'y': float(pose.pose.position.y),
                'yaw': values[2] - oyaw,
                'frame_id': pose.header.frame_id,
            },
        }

    def _reset_leg_events(self, ctx) -> None:
        ctx.finish_event.clear()
        ctx.finish_terminal = None
        ctx.nav_arrived_event.clear()

    def _run_nav_leg(
        self,
        *,
        goal_handle,
        ctx,
        mission_id: str,
        generation: int,
        station_id: str,
        pose=None,
        poses=None,
    ) -> Optional[MissionTerminal]:
        """Navigate one ToPose or one ThroughPoses leg.

        Returns None on arrival; terminal on fail/cancel.
        Vias are never inspection stations — only the destination station_id
        is used for feedback / later capture.
        """
        self._reset_leg_events(ctx)

        def on_feedback(mid, sid, phase, msg):
            if not self.state.is_owner(mid, generation):
                return
            self._publish_feedback(goal_handle, mid, sid, phase, msg)

        def on_finished(terminal: MissionTerminal):
            ctx.publish_finished(terminal)

        # Motion-before mark (robot-level safety JSON). Failure → bounded fail, keep refuse.
        mark_ok, mark_msg = self._guard.mark_active(mission_id=mission_id)
        if not mark_ok:
            self.state.set_safety_refuse(True, f'motion safety mark failed: {mark_msg}')
            return MissionTerminal(
                mission_id=mission_id,
                execution_status=ExecutionStatus.FAILED.value,
                error_code=ErrorCode.SAFETY_REFUSE,
                message=f'motion blocked: safety mark write failed: {mark_msg}',
                generation=generation,
                station_id=station_id,
            )

        runner = MissionRunner(
            state=self.state,
            nav=self._nav,
            pose=pose,
            poses=poses,
            mission_id=mission_id,
            generation=generation,
            station_id=station_id,
            server_timeout_sec=float(self.get_parameter('server_timeout_sec').value),
            send_timeout_sec=float(self.get_parameter('send_timeout_sec').value),
            result_timeout_sec=float(self.get_parameter('result_timeout_sec').value),
            cancel_timeout_sec=float(self.get_parameter('cancel_timeout_sec').value),
            on_feedback=on_feedback,
            on_finished=on_finished,
            defer_success=True,
            nav_arrived_event=ctx.nav_arrived_event,
            managed_occupancy=True,
        )
        pending = False
        with self.state.lock:
            ctx.runner = runner
            pending = bool(ctx.pending_cancel)
            if pending:
                ctx.pending_cancel = False
        if pending:
            runner.request_cancel()
        runner.start()

        while rclpy.ok():
            if ctx.finish_event.is_set():
                runner.mark_externally_finished()
                return ctx.finish_terminal
            if ctx.nav_arrived_event.is_set():
                runner.mark_externally_finished()
                return None
            if ctx.pending_cancel or runner.user_cancel_requested():
                runner.request_cancel()
            ctx.finish_event.wait(timeout=0.1)
        return MissionTerminal(
            mission_id=mission_id,
            execution_status=ExecutionStatus.FAILED.value,
            error_code=ErrorCode.INTERNAL_ERROR,
            message='rclpy shutdown during navigation',
            generation=generation,
            station_id=station_id,
        )

    def _capture_and_inspect(
        self,
        *,
        goal_handle,
        ctx,
        runner: MissionRunner,
        mission_id: str,
        generation: int,
        station_id: str,
        route_id: str,
    ) -> StationWorkResult:
        def should_abort() -> bool:
            return (
                ctx.pending_cancel
                or runner.user_cancel_requested()
                or not self.state.is_owner(mission_id, generation)
            )

        def on_phase(phase: str, message: str) -> None:
            if not self.state.is_owner(mission_id, generation):
                return
            self.state.set_phase(phase, mission_id=mission_id, generation=generation)
            self._publish_feedback(goal_handle, mission_id, station_id, phase, message)

        if should_abort():
            return StationWorkResult(
                ok=False, canceled=True, station_id=station_id,
                message='canceled after arrival before capture',
            )

        capture = capture_station_frame(
            camera=self._camera,
            settle=self._settle,
            output_dir=Path(self.get_parameter('capture_output_dir').value),
            mission_id=mission_id,
            station_id=station_id,
            route_id=route_id,
            settle_timeout_sec=float(self.get_parameter('settle_timeout_sec').value),
            settle_sec=float(self.get_parameter('settle_sec').value),
            frame_timeout_sec=float(self.get_parameter('frame_timeout_sec').value),
            should_abort=should_abort,
            on_phase=on_phase,
            nav_goal=self._nav_goal_diag(station_id),
            pose_snapshotter=self._poses,
        )

        if capture.error_code == 'canceled' or should_abort():
            return StationWorkResult(
                ok=False, canceled=True, station_id=station_id,
                message=capture.message or 'canceled during settle/capture',
                screenshot_path=capture.screenshot_path or '',
                image_stamp=capture.image_stamp or '',
                frame_id=capture.frame_id or '',
            )
        if not capture.ok:
            return StationWorkResult(
                ok=False, station_id=station_id,
                error_code=capture.error_code,
                message=capture.message,
            )

        if should_abort():
            return StationWorkResult(
                ok=False, canceled=True, station_id=station_id,
                message='canceled after capture before inspect',
                screenshot_path=capture.screenshot_path,
                image_stamp=capture.image_stamp,
                frame_id=capture.frame_id,
            )

        on_phase(Phase.INSPECTING.value, f'same-frame YOLO+ROI inspection at {station_id}')
        yaw = float(self._route['stations'][station_id][2])
        gt = (capture.diagnostics or {}).get('gt_world_pose') or {}
        if isinstance(gt, dict) and 'yaw' in gt:
            yaw = float(gt['yaw'])
        inspect = inspect_station_image(
            screenshot_path=Path(capture.screenshot_path),
            mission_id=mission_id,
            station_id=station_id,
            image_stamp=capture.image_stamp,
            robot_yaw=yaw,
            weights=Path(self.get_parameter('yolo_weights').value),
            rules_path=Path(self.get_parameter('roi_rules').value),
            conf=float(self.get_parameter('yolo_conf').value),
        )
        if should_abort():
            return StationWorkResult(
                ok=False, canceled=True, station_id=station_id,
                message='canceled during/after inspect',
                screenshot_path=capture.screenshot_path,
                image_stamp=capture.image_stamp,
                frame_id=capture.frame_id,
                inspection_status=inspect.inspection_status,
                station_result_path=inspect.station_result_path,
                model_version=inspect.model_version,
            )
        if not inspect.ok:
            return StationWorkResult(
                ok=False, station_id=station_id,
                error_code=ErrorCode.VISION_FAILED,
                message=inspect.message,
                screenshot_path=capture.screenshot_path,
                image_stamp=capture.image_stamp,
                frame_id=capture.frame_id,
                model_version=inspect.model_version,
            )

        # Reuse InspectResult.payload; missing/empty objects → execution failure.
        payload = inspect.payload if isinstance(inspect.payload, dict) else {}
        objects = payload.get('objects')
        if not isinstance(objects, dict) or not objects:
            return StationWorkResult(
                ok=False, station_id=station_id,
                error_code=ErrorCode.VISION_FAILED,
                message='inspect payload missing objects; refusing empty success archive',
                screenshot_path=capture.screenshot_path,
                image_stamp=capture.image_stamp,
                frame_id=capture.frame_id,
                inspection_status=inspect.inspection_status or '',
                station_result_path=inspect.station_result_path,
                model_version=inspect.model_version,
            )

        return StationWorkResult(
            ok=True,
            station_id=station_id,
            message=capture.message,
            screenshot_path=capture.screenshot_path,
            image_stamp=capture.image_stamp,
            frame_id=capture.frame_id,
            inspection_status=inspect.inspection_status,
            station_result_path=inspect.station_result_path,
            model_version=inspect.model_version,
            annotated_path=str(
                payload.get('annotated_path') or inspect.annotated_path or '',
            ),
            roi_rules_version=str(payload.get('roi_rules_version') or ''),
            objects=objects,
        )

    def _persist_station_work(
        self, mission_id: str, work: StationWorkResult,
    ) -> Optional[str]:
        """Persist one station. Returns error message or None."""
        wr = self._archive.persist_station(
            mission_id=mission_id,
            station={
                'station_id': work.station_id,
                'inspection_status': work.inspection_status,
                'image_stamp': work.image_stamp,
                'screenshot_path': work.screenshot_path,
                'annotated_path': work.annotated_path,
                'station_result_path': work.station_result_path,
                'model_version': work.model_version,
                'roi_rules_version': work.roi_rules_version,
                'objects': work.objects,
            },
        )
        if wr.ok:
            self.get_logger().info(
                f'archived station mission={mission_id} station={work.station_id} '
                f'insp={work.inspection_status}',
            )
            return None
        self.get_logger().error(
            f'archive station failed: {wr.error_code} {wr.message}',
        )
        return f'{wr.error_code}: {wr.message}'

    def _make_terminal(
        self,
        *,
        mission_id: str,
        generation: int,
        execution_status: str,
        error_code: str,
        message: str,
        station_id: str = '',
        screenshot_path: str = '',
        image_stamp: str = '',
        frame_id: str = '',
        inspection_status: str = '',
        station_result_path: str = '',
        model_version: str = '',
    ) -> MissionTerminal:
        return MissionTerminal(
            mission_id=mission_id,
            execution_status=execution_status,
            error_code=error_code,
            message=message,
            generation=generation,
            screenshot_path=screenshot_path,
            image_stamp=image_stamp,
            frame_id=frame_id,
            station_id=station_id,
            inspection_status=inspection_status,
            station_result_path=station_result_path,
            model_version=model_version,
        )

    def _seal_mission_terminal(
        self,
        *,
        ctx,
        terminal: MissionTerminal,
        release: bool,
        mark_unsafe: bool = False,
        already_published: bool = False,
    ) -> MissionTerminal:
        """Hold occupancy → archive → clear owner mark → then atomic release (R2).

        already_published is ignored for occupancy: Runtime always claims/holds
        until seal completes. Clear failure keeps refuse and does not open slots.
        """
        del already_published  # occupancy always Runtime-owned until seal
        if self._archive is None:
            self.state.set_safety_refuse(True, 'seal without archive writer')
            return self._make_terminal(
                mission_id=terminal.mission_id,
                generation=int(terminal.generation or ctx.generation),
                execution_status=ExecutionStatus.FAILED.value,
                error_code=ErrorCode.ARCHIVE_FAILED,
                message='archive writer unavailable at seal',
                station_id=terminal.station_id,
            )
        generation = terminal.generation
        if generation is None:
            generation = ctx.generation
        generation = int(generation)
        claimed = self.state.claim_finalizing_terminal(
            terminal.mission_id,
            terminal.execution_status,
            terminal.error_code,
            terminal.message,
            generation=generation,
            mark_unsafe=mark_unsafe,
            screenshot_path=terminal.screenshot_path,
            image_stamp=terminal.image_stamp,
            frame_id=terminal.frame_id,
            station_id=terminal.station_id,
            inspection_status=terminal.inspection_status,
            station_result_path=terminal.station_result_path,
            model_version=terminal.model_version,
        )
        if claimed is not None:
            terminal = claimed
        elif not self.state.is_owner(terminal.mission_id, generation):
            # Lost ownership — do not clear foreign safety / rewrite state.
            return terminal

        # Still busy here.
        assert self.state.is_busy

        wr = self._archive.finalize_mission(
            mission_id=terminal.mission_id,
            execution_status=terminal.execution_status,
            error_code=terminal.error_code,
            message=terminal.message,
            ended_at=utc_now_iso(),
        )
        if not wr.ok:
            self.get_logger().error(
                f'finalize archive failed: {wr.error_code} {wr.message}',
            )
            failed = self._make_terminal(
                mission_id=terminal.mission_id,
                generation=generation,
                execution_status=ExecutionStatus.FAILED.value,
                error_code=ErrorCode.ARCHIVE_FAILED,
                message=(
                    f'archive not committed ({wr.error_code}: {wr.message}); '
                    f'prior_status={terminal.execution_status}; {terminal.message}'
                ),
                station_id=terminal.station_id,
                screenshot_path=terminal.screenshot_path,
                image_stamp=terminal.image_stamp,
                frame_id=terminal.frame_id,
                inspection_status=terminal.inspection_status,
                station_result_path=terminal.station_result_path,
                model_version=terminal.model_version,
            )
            self.state.commit_archive_result(
                terminal.mission_id,
                generation,
                archive_ok=False,
                mark_unsafe=True,
                override_terminal=failed,
            )
            self.state.set_safety_refuse(
                True, f'archive_degraded: {wr.error_code}: {wr.message}',
            )
            return failed

        out = self.state.commit_archive_result(
            terminal.mission_id,
            generation,
            archive_ok=True,
            mark_unsafe=mark_unsafe,
            override_terminal=terminal,
        ) or terminal

        if mark_unsafe or not release:
            self._guard.mark_refuse(
                reason=out.error_code or 'unsafe',
                mission_id=out.mission_id,
            )
            self.state.release_occupancy_if_owner(
                out.mission_id, generation, mark_unsafe=True,
            )
            return out

        # Clear while still logically occupied; only then open new tasks.
        clear_ok, clear_msg = self._guard.clear_mark_if_owner(
            instance_id=self._guard.instance_id,
            mission_id=out.mission_id,
        )
        if not clear_ok:
            self.state.set_safety_refuse(
                True, f'safety clear failed after archive: {clear_msg}',
            )
            self.get_logger().error(
                f'archive ok but safety clear failed; keeping occupancy refuse: '
                f'{clear_msg}',
            )
            # Business archive succeeded; refuse new goals without pretending DB fail.
            return out

        self.state.release_occupancy_if_owner(
            out.mission_id, generation, mark_unsafe=False,
        )
        return out

    def _publish_once(
        self,
        *,
        mission_id: str,
        generation: int,
        execution_status: str,
        error_code: str,
        message: str,
        station_id: str = '',
        screenshot_path: str = '',
        image_stamp: str = '',
        frame_id: str = '',
        inspection_status: str = '',
        station_result_path: str = '',
        model_version: str = '',
        release: bool = True,
        mark_unsafe: bool = False,
    ) -> MissionTerminal:
        """Legacy helper: immediate state publish (preflight / early rejects only)."""
        terminal = self.state.try_publish_terminal(
            mission_id,
            execution_status,
            error_code,
            message,
            release=release,
            mark_unsafe=mark_unsafe,
            generation=generation,
            screenshot_path=screenshot_path,
            image_stamp=image_stamp,
            frame_id=frame_id,
            station_id=station_id,
            inspection_status=inspection_status,
            station_result_path=station_result_path,
            model_version=model_version,
        )
        if terminal is not None:
            return terminal
        return self._make_terminal(
            mission_id=mission_id,
            generation=generation,
            execution_status=execution_status,
            error_code=error_code,
            message=message,
            station_id=station_id,
            screenshot_path=screenshot_path,
            image_stamp=image_stamp,
            frame_id=frame_id,
            inspection_status=inspection_status,
            station_result_path=station_result_path,
            model_version=model_version,
        )

    def _execute_callback(self, goal_handle):
        request = goal_handle.request
        mission_id = str(request.mission_id).strip()
        route_id = str(getattr(request, 'route_id', '') or ALLOWED_ROUTE_ID).strip()
        ctx = self.slot.get_by_goal_handle(goal_handle)
        if ctx is None:
            result = ExecuteInspection.Result()
            result.mission_id = mission_id
            result.execution_status = ExecutionStatus.FAILED.value
            result.error_code = ErrorCode.INTERNAL_ERROR
            result.message = 'missing mission context'
            goal_handle.abort()
            return result

        generation = ctx.generation
        ctx.started_at = utc_now_iso()
        ctx.route_id = route_id

        try:
            station_ids = stations_for_route(route_id, self._route)
        except ValueError as exc:
            terminal = self._publish_once(
                mission_id=mission_id, generation=generation,
                execution_status=ExecutionStatus.FAILED.value,
                error_code=ErrorCode.UNSUPPORTED_ROUTE,
                message=str(exc),
            )
            self.clear_active_context_if_owner(ctx)
            result = ExecuteInspection.Result()
            self._fill_result(result, terminal)
            goal_handle.abort()
            return result

        if self._archive is None:
            terminal = self._publish_once(
                mission_id=mission_id, generation=generation,
                execution_status=ExecutionStatus.FAILED.value,
                error_code=ErrorCode.ARCHIVE_FAILED,
                message='archive writer disabled',
            )
            self.clear_active_context_if_owner(ctx)
            result = ExecuteInspection.Result()
            self._fill_result(result, terminal)
            goal_handle.abort()
            return result

        wr0 = self._archive.begin_mission(
            mission_id=mission_id, route_id=route_id, started_at=ctx.started_at,
        )
        if not wr0.ok:
            terminal = self._publish_once(
                mission_id=mission_id, generation=generation,
                execution_status=ExecutionStatus.FAILED.value,
                error_code=ErrorCode.ARCHIVE_FAILED,
                message=f'begin_mission failed: {wr0.error_code}: {wr0.message}',
            )
            self.clear_active_context_if_owner(ctx)
            result = ExecuteInspection.Result()
            self._fill_result(result, terminal)
            goal_handle.abort()
            return result

        insp_statuses: list[str] = []
        completed_stations: list[str] = []
        last_work: Optional[StationWorkResult] = None
        terminal: Optional[MissionTerminal] = None
        release = True
        mark_unsafe = False
        prev_station: Optional[str] = None
        had_nav_attempt = False

        try:
            for station_id in station_ids:
                if ctx.pending_cancel:
                    terminal = self._make_terminal(
                        mission_id=mission_id, generation=generation,
                        execution_status=ExecutionStatus.CANCELED.value,
                        error_code=ErrorCode.NONE,
                        message=(
                            f'canceled before station {station_id}; '
                            f'completed_stations={completed_stations}'
                        ),
                        station_id=station_id,
                        inspection_status=aggregate_inspection_status(insp_statuses),
                    )
                    break

                # Patrol baseline: no vias → NavigateToPose; vias → one
                # NavigateThroughPoses(vias + goal). Capture only after goal station.
                through_world = []
                if prev_station is not None:
                    through_world = transfer_through_world_poses(
                        self._route,
                        from_station=prev_station,
                        to_station=station_id,
                    )
                if through_world:
                    stamp = self.get_clock().now().to_msg()
                    through_poses = [
                        pose_from_route_values(vals, self._origin, stamp=stamp)
                        for vals in through_world
                    ]
                    transfer_label = f'{prev_station}_to_{station_id}'
                    self._publish_feedback(
                        goal_handle,
                        mission_id,
                        station_id,
                        Phase.NAVIGATING.value,
                        (
                            f'{transfer_label}: NavigateThroughPoses '
                            f'n={len(through_poses)} (vias+goal)'
                        ),
                    )
                    nav_term = self._run_nav_leg(
                        goal_handle=goal_handle,
                        ctx=ctx,
                        mission_id=mission_id,
                        generation=generation,
                        station_id=station_id,
                        poses=through_poses,
                    )
                else:
                    nav_term = self._run_nav_leg(
                        goal_handle=goal_handle,
                        ctx=ctx,
                        mission_id=mission_id,
                        generation=generation,
                        station_id=station_id,
                        pose=self._station_pose(station_id),
                    )
                had_nav_attempt = True
                if nav_term is not None:
                    unsafe = (
                        nav_term.error_code == ErrorCode.CANCEL_UNCONFIRMED
                        or bool(self.state.unsafe_busy)
                    )
                    msg = (
                        f'nav ended before capture at {station_id}: {nav_term.message}; '
                        f'completed_stations={completed_stations}'
                    )
                    mark_unsafe = unsafe
                    release = not mark_unsafe
                    terminal = self._make_terminal(
                        mission_id=mission_id, generation=generation,
                        execution_status=nav_term.execution_status,
                        error_code=nav_term.error_code,
                        message=msg,
                        station_id=station_id,
                        inspection_status=aggregate_inspection_status(insp_statuses),
                    )
                    break

                runner = ctx.runner
                work = self._capture_and_inspect(
                    goal_handle=goal_handle,
                    ctx=ctx,
                    runner=runner,
                    mission_id=mission_id,
                    generation=generation,
                    station_id=station_id,
                    route_id=route_id,
                )
                last_work = work

                if work.canceled:
                    terminal = self._make_terminal(
                        mission_id=mission_id, generation=generation,
                        execution_status=ExecutionStatus.CANCELED.value,
                        error_code=ErrorCode.NONE,
                        message=(
                            f'{work.message}; completed_stations={completed_stations}'
                        ),
                        station_id=station_id,
                        screenshot_path=work.screenshot_path,
                        image_stamp=work.image_stamp,
                        frame_id=work.frame_id,
                        inspection_status=aggregate_inspection_status(insp_statuses),
                        station_result_path=work.station_result_path,
                        model_version=work.model_version,
                    )
                    break

                if not work.ok:
                    if work.screenshot_path:
                        self._persist_station_work(mission_id, work)
                    terminal = self._make_terminal(
                        mission_id=mission_id, generation=generation,
                        execution_status=ExecutionStatus.FAILED.value,
                        error_code=work.error_code or ErrorCode.INTERNAL_ERROR,
                        message=(
                            f'station {station_id} failed: {work.message}; '
                            f'completed_stations={completed_stations}'
                        ),
                        station_id=station_id,
                        screenshot_path=work.screenshot_path,
                        image_stamp=work.image_stamp,
                        frame_id=work.frame_id,
                        inspection_status=aggregate_inspection_status(insp_statuses),
                        station_result_path=work.station_result_path,
                        model_version=work.model_version,
                    )
                    break

                # Inspection FAIL/UNKNOWN does NOT block next station.
                arch_err = self._persist_station_work(mission_id, work)
                if arch_err:
                    terminal = self._make_terminal(
                        mission_id=mission_id, generation=generation,
                        execution_status=ExecutionStatus.FAILED.value,
                        error_code=ErrorCode.ARCHIVE_FAILED,
                        message=(
                            f'station {station_id} archive failed: {arch_err}; '
                            f'completed_stations={completed_stations}'
                        ),
                        station_id=station_id,
                        screenshot_path=work.screenshot_path,
                        image_stamp=work.image_stamp,
                        frame_id=work.frame_id,
                        inspection_status=work.inspection_status,
                        station_result_path=work.station_result_path,
                        model_version=work.model_version,
                    )
                    # Station persist failed — keep refuse after seal.
                    mark_unsafe = True
                    release = False
                    break

                completed_stations.append(station_id)
                insp_statuses.append(work.inspection_status)
                prev_station = station_id
                self._publish_feedback(
                    goal_handle, mission_id, station_id, Phase.INSPECTING.value,
                    (
                        f'station {station_id} done insp={work.inspection_status}; '
                        f'continuing route'
                    ),
                )
            if terminal is None:
                agg = aggregate_inspection_status(insp_statuses)
                msg = (
                    f'route {route_id} completed stations={station_ids}; '
                    f'inspection_statuses={insp_statuses}; '
                    f'aggregate_inspection={agg} '
                    f'(rule: FAIL>UNKNOWN>PASS; UNKNOWN never masks FAIL)'
                )
                lw = last_work
                terminal = self._make_terminal(
                    mission_id=mission_id, generation=generation,
                    execution_status=ExecutionStatus.COMPLETED.value,
                    error_code=ErrorCode.NONE,
                    message=msg,
                    station_id=(lw.station_id if lw else ''),
                    screenshot_path=(lw.screenshot_path if lw else ''),
                    image_stamp=(lw.image_stamp if lw else ''),
                    frame_id=(lw.frame_id if lw else ''),
                    inspection_status=agg,
                    station_result_path=(lw.station_result_path if lw else ''),
                    model_version=(lw.model_version if lw else ''),
                )
        except Exception as exc:  # noqa: BLE001 — bounded mission exit (R3)
            self.get_logger().error(f'execute loop exception: {exc}')
            if had_nav_attempt and not self.state.unsafe_busy:
                # Possible in-flight nav — keep refuse/unsafe rather than free release.
                mark_unsafe = True
                release = False
                runner = ctx.runner
                if runner is not None:
                    try:
                        runner.request_cancel()
                    except Exception:
                        pass
            terminal = self._make_terminal(
                mission_id=mission_id,
                generation=generation,
                execution_status=ExecutionStatus.FAILED.value,
                error_code=(
                    ErrorCode.CANCEL_UNCONFIRMED if mark_unsafe
                    else ErrorCode.INTERNAL_ERROR
                ),
                message=(
                    f'bounded execute exception: {exc}; '
                    f'completed_stations={completed_stations}'
                ),
                inspection_status=aggregate_inspection_status(insp_statuses),
            )

        if terminal is None:
            terminal = self._make_terminal(
                mission_id=mission_id,
                generation=generation,
                execution_status=ExecutionStatus.FAILED.value,
                error_code=ErrorCode.INTERNAL_ERROR,
                message='execute ended without terminal',
            )
            release = False
            mark_unsafe = True
        if terminal.mission_id != mission_id:
            terminal = self._make_terminal(
                mission_id=mission_id,
                generation=generation,
                execution_status=ExecutionStatus.FAILED.value,
                error_code=ErrorCode.INTERNAL_ERROR,
                message='terminal ownership mismatch',
            )
            release = False
            mark_unsafe = True

        # Seal while context/timer still available; clear context after.
        try:
            terminal = self._seal_mission_terminal(
                ctx=ctx,
                terminal=terminal,
                release=release,
                mark_unsafe=mark_unsafe,
            )
        finally:
            self.clear_active_context_if_owner(ctx)

        result = ExecuteInspection.Result()
        self._fill_result(result, terminal)

        status = terminal.execution_status
        if status == ExecutionStatus.COMPLETED.value:
            goal_handle.succeed()
        elif status == ExecutionStatus.CANCELED.value:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.get_logger().info(
            f"ExecuteInspection done mission_id={result.mission_id} "
            f"route={route_id} status={result.execution_status} "
            f"code={result.error_code} insp={result.inspection_status}",
        )
        return result

    def safety_status(self) -> dict[str, Any]:
        snap = self._guard.read_safety()
        return {
            'ok': True,
            'robot_id': self._guard.robot_id,
            'instance_id': self._guard.instance_id,
            'flock_held': self._guard.held,
            'flock_failed': self._guard.flock_failed,
            'writer_enabled': self._writer_enabled,
            'runtime_dir': str(self._guard.runtime_dir),
            'safety_path': str(self._guard.safety_path),
            'safety_ok': snap.ok,
            'safety_refuse': snap.refuse,
            'safety_reason': snap.reason,
            'state_safety_refuse': self.state.safety_refuse,
            'state_unsafe_busy': self.state.unsafe_busy,
            'state_archive_degraded': self.state.archive_degraded,
            'busy': self.state.is_busy,
            'finalizing': self.state.finalizing,
        }

    def _ipc_status(self) -> dict[str, Any]:
        return self.safety_status()

    def _ipc_arm(self) -> dict[str, Any]:
        if self._guard.flock_failed or not self._guard.held:
            return {
                'ok': False,
                'message': 'cannot arm: flock not held (irreversible without restart)',
            }
        token = self.arm_operator_confirm()
        return {
            'ok': True,
            'instance_id': self._guard.instance_id,
            'token': token,
            'message': 'armed on live Runtime',
        }

    def _ipc_confirm(self, instance_id: str, token: str) -> dict[str, Any]:
        ok, msg, interrupted = self.operator_confirm(
            instance_id=instance_id, token=token, interrupt_running=True,
        )
        return {
            'ok': ok,
            'message': msg,
            'interrupted': interrupted,
            'instance_id': self._guard.instance_id,
        }

    def arm_operator_confirm(self) -> str:
        """Return one-shot token bound to current instance_id."""
        return self._guard.arm_operator_confirm(persist=False)

    def operator_confirm(
        self,
        *,
        instance_id: str,
        token: str,
        interrupt_running: bool = True,
    ) -> tuple[bool, str, list[str]]:
        """Ordered recovery: consume token → interrupt archive → clear → barriers.

        Does not auto-resume, cancel foreign Nav2, or reset initialpose.
        Flock-fail barrier cannot be cleared by confirm.
        """
        if self._guard.flock_failed or not self._guard.held:
            return False, 'flock failure barrier cannot be cleared by confirm', []
        ok_idle, idle_msg = self.state.can_accept_operator_confirm()
        if not ok_idle:
            return False, idle_msg, []
        if not self.state.claim_recovery():
            return False, 'could not claim recovery', []
        interrupted: list[str] = []
        try:
            tok_ok, tok_msg = self._guard.validate_and_consume_token(
                instance_id=instance_id, token=token, allow_persisted=False,
            )
            if not tok_ok:
                return False, tok_msg, []
            # Explicit recovery takeover BEFORE archive interrupt / clear.
            # Does not impersonate old owner; normal seal owner checks stay strict.
            take_ok, take_msg, _audit = self._guard.takeover_stale_mark_for_recovery()
            if not take_ok:
                self.state.set_safety_refuse(
                    True, f'recovery takeover failed: {take_msg}',
                )
                return False, f'takeover failed: {take_msg}', []
            if interrupt_running:
                if self._archive is None:
                    self.state.set_safety_refuse(True, 'confirm without archive writer')
                    return False, 'archive writer unavailable', []
                interrupted, err = self._archive.interrupt_running_missions(
                    reason=(
                        f'operator confirm instance_id={self._guard.instance_id}; '
                        'Runtime sole writer closed RUNNING shell as INTERRUPTED'
                    ),
                )
                if err:
                    self.state.set_safety_refuse(True, err)
                    return False, err, interrupted
            clear_ok, clear_msg = self._guard.clear_mark_if_owner(
                instance_id=self._guard.instance_id,
            )
            if not clear_ok:
                self.state.set_safety_refuse(
                    True, f'confirm token ok but clear failed: {clear_msg}',
                )
                return False, f'clear failed: {clear_msg}', interrupted
            self.state.operator_clear_barriers()
            return True, 'confirmed', interrupted
        finally:
            self.state.clear_recovery_claim()

    def shutdown_runtime(self) -> None:
        try:
            if self._archive is not None:
                self._archive.close()
        except Exception:
            pass
        try:
            self._guard.close()
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionRuntimeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node.shutdown_runtime()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
