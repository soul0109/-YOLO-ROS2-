"""Single-process ROS2 ActionClient bridge for /inspection/execute.

One Node + one executor thread for the FastAPI process lifetime.
Does not publish /cmd_vel or call Nav2/vision directly.

V2-02 observation contract:
- observation_status is independent of Action execution_status.
- HTTP 504 / late accept → PENDING (keep tracking; attach result once).
- Result communication errors → UNKNOWN (do not forge robot FAILED).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Optional

# rclpy / action imports happen in start() so offline unit tests can mock.

ALLOWED_ROUTE_ID = 'test_room_A'  # default create
ALLOWED_ROUTE_IDS = frozenset({'test_room_A', 'test_room_ABC'})

OBS_CLEAR = 'CLEAR'
OBS_PENDING = 'PENDING'
OBS_UNKNOWN = 'UNKNOWN'


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


@dataclass
class LiveMissionState:
    mission_id: str
    route_id: str
    status: str = 'ACCEPTED'  # ACCEPTED|RUNNING|COMPLETED|FAILED|CANCELED|INTERRUPTED
    phase: str = 'VALIDATING'
    error_code: str = 'NONE'
    message: str = ''
    inspection_status: str = ''
    station_id: str = ''
    screenshot_path: str = ''
    station_result_path: str = ''
    model_version: str = ''
    image_stamp: str = ''
    started_at: str = field(default_factory=_utc_now)
    ended_at: str = ''
    terminal: bool = False
    observation_status: str = OBS_CLEAR


class RosMissionBridge:
    """Owns rclpy lifecycle and live mission tracking for HTTP."""

    def __init__(
        self,
        *,
        action_name: str = '/inspection/execute',
        server_wait_sec: float = 2.0,
        goal_response_timeout_sec: float = 15.0,
    ) -> None:
        self.action_name = action_name
        self.server_wait_sec = server_wait_sec
        self.goal_response_timeout_sec = goal_response_timeout_sec

        self._lock = threading.RLock()
        self._live: dict[str, LiveMissionState] = {}
        self._busy_mission_id: Optional[str] = None
        # mission_id → True once get_result_async callback attached
        self._result_attached: dict[str, bool] = {}
        self._pending_send: dict[str, Any] = {}

        self._rclpy = None
        self._node = None
        self._client = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._started = False
        # Read-only console telemetry (optional; attached after node start).
        from .console_hub import ConsoleHub
        self.console = ConsoleHub()

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> None:
        if self._started:
            return
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from inspection_interfaces.action import ExecuteInspection

        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self._node = Node('mission_api_bridge')
        self._client = ActionClient(
            self._node, ExecuteInspection, self.action_name,
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._spin, name='mission-api-ros-spin', daemon=True,
        )
        self._spin_thread.start()
        self._started = True
        try:
            self.console.attach_node(self._node)
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warn(f'ConsoleHub attach failed: {exc}')
        self._node.get_logger().info(
            f'Mission API ROS bridge ready action={self.action_name}',
        )

    def _spin(self) -> None:
        assert self._executor is not None
        try:
            self._executor.spin()
        except Exception:
            pass

    def stop(self) -> None:
        if not self._started:
            return
        self._started = False
        try:
            if self._executor is not None:
                self._executor.shutdown()
        except Exception:
            pass
        try:
            self.console.detach()
        except Exception:
            pass
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
        try:
            if self._rclpy is not None and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=3.0)
        self._node = None
        self._client = None
        self._executor = None
        self._spin_thread = None

    def is_action_server_ready(self) -> bool:
        if not self._started or self._client is None:
            return False
        return bool(self._client.server_is_ready())

    def wait_for_server(self, timeout_sec: Optional[float] = None) -> bool:
        if not self._started or self._client is None:
            return False
        t = self.server_wait_sec if timeout_sec is None else timeout_sec
        return bool(self._client.wait_for_server(timeout_sec=t))

    def get_live(self, mission_id: str) -> Optional[LiveMissionState]:
        with self._lock:
            st = self._live.get(mission_id)
            return None if st is None else LiveMissionState(**st.__dict__)

    def list_live(self) -> list[LiveMissionState]:
        with self._lock:
            return [LiveMissionState(**s.__dict__) for s in self._live.values()]

    def is_bridge_busy(self) -> bool:
        with self._lock:
            return self._busy_mission_id is not None

    def _clear_busy_if(self, mission_id: str) -> None:
        if self._busy_mission_id == mission_id:
            self._busy_mission_id = None

    def reconcile_from_archive_terminal(
        self,
        mission_id: str,
        *,
        execution_status: str,
        error_code: str = 'NONE',
        message: str = '',
    ) -> None:
        """One-way reconcile: authoritative archive terminal clears matching live busy.

        Does not clear other missions. Does not claim Runtime safety barriers are clear.
        """
        mission_id = str(mission_id or '').strip()
        status = str(execution_status or '')
        if not mission_id or not status or status == 'RUNNING':
            return
        with self._lock:
            st = self._live.get(mission_id)
            if st is None:
                self._clear_busy_if(mission_id)
                return
            if st.terminal:
                self._clear_busy_if(mission_id)
                st.observation_status = OBS_CLEAR
                return
            st.status = status
            st.error_code = str(error_code or 'NONE')
            if message:
                st.message = message
            st.phase = ''
            st.terminal = True
            st.observation_status = OBS_CLEAR
            st.ended_at = st.ended_at or _utc_now()
            self._clear_busy_if(mission_id)

    def _mark_observation_unknown(self, mission_id: str, *, error_code: str, message: str) -> None:
        st = self._live.get(mission_id)
        if st is not None and not st.terminal:
            st.observation_status = OBS_UNKNOWN
            st.error_code = error_code
            st.message = message
        # Unaccepted / unknown communication: free local busy so POST is not forever 409.
        self._clear_busy_if(mission_id)

    def _finish_unaccepted_reject(self, mission_id: str, *, message: str) -> None:
        """Late explicit reject after 504: end tracking and clear matching busy."""
        st = self._live.get(mission_id)
        if st is not None and not st.terminal:
            st.observation_status = OBS_CLEAR
            st.error_code = 'goal_rejected'
            st.message = message
            st.status = 'REJECTED'
            st.terminal = True
            st.ended_at = _utc_now()
        self._live.pop(mission_id, None)
        self._clear_busy_if(mission_id)
        self._result_attached.pop(mission_id, None)
        self._pending_send.pop(mission_id, None)

    def _attach_result_once(self, mission_id: str, route_id: str, goal_handle: Any) -> None:
        """Attach get_result_async at most once for this mission_id."""
        with self._lock:
            if self._result_attached.get(mission_id):
                return
            self._result_attached[mission_id] = True

        def _result_cb(future) -> None:
            try:
                result_wrap = future.result()
                result = result_wrap.result
            except Exception as exc:
                with self._lock:
                    self._mark_observation_unknown(
                        mission_id,
                        error_code='result_communication_error',
                        message=f'result future failed (observation): {exc}',
                    )
                return

            with self._lock:
                self._apply_terminal_from_result(mission_id, route_id, result)

        try:
            result_future = goal_handle.get_result_async()
        except Exception as exc:
            with self._lock:
                self._mark_observation_unknown(
                    mission_id,
                    error_code='result_communication_error',
                    message=f'get_result_async failed (observation): {exc}',
                )
            return
        result_future.add_done_callback(_result_cb)

    def _apply_terminal_from_result(self, mission_id: str, route_id: str, result: Any) -> None:
        """Update live state from action result. Never creates ACCEPTED over terminal."""
        st = self._live.get(mission_id)
        if st is None:
            st = LiveMissionState(mission_id=mission_id, route_id=route_id)
            self._live[mission_id] = st
        if st.terminal:
            self._clear_busy_if(mission_id)
            st.observation_status = OBS_CLEAR
            return
        st.status = str(getattr(result, 'execution_status', None) or 'FAILED')
        st.error_code = str(getattr(result, 'error_code', None) or 'NONE')
        st.message = str(getattr(result, 'message', None) or '')
        st.inspection_status = str(getattr(result, 'inspection_status', None) or '')
        sid = str(getattr(result, 'station_id', None) or '')
        if sid:
            st.station_id = sid
        st.screenshot_path = str(getattr(result, 'screenshot_path', None) or '')
        st.station_result_path = str(getattr(result, 'station_result_path', None) or '')
        st.model_version = str(getattr(result, 'model_version', None) or '')
        st.image_stamp = str(getattr(result, 'image_stamp', None) or '')
        st.phase = ''
        st.terminal = True
        st.observation_status = OBS_CLEAR
        st.ended_at = _utc_now()
        self._clear_busy_if(mission_id)
        try:
            self.console.set_current_goal(None)
        except Exception:
            pass

    def submit_goal(
        self,
        mission_id: str,
        route_id: str,
        *,
        archived_ids: Optional[Callable[[], set[str]]] = None,
    ) -> tuple[int, str, str, Optional[LiveMissionState]]:
        """Send ExecuteInspection goal; wait only for accept/reject.

        Returns (http_status, error_code, message, live_state_or_none).
        """
        mission_id = str(mission_id or '').strip()
        route_id = str(route_id or '').strip()

        if not mission_id:
            return 400, 'empty_mission_id', 'mission_id must be non-empty', None
        if route_id not in ALLOWED_ROUTE_IDS:
            return (
                400,
                'unsupported_route',
                f"route_id must be one of {sorted(ALLOWED_ROUTE_IDS)}, got '{route_id}'",
                None,
            )

        if archived_ids is not None:
            try:
                if mission_id in archived_ids():
                    return (
                        409,
                        'duplicate_mission_id',
                        f"mission_id '{mission_id}' already archived",
                        None,
                    )
            except Exception as exc:
                return 503, 'archive_unavailable', f'cannot read archive: {exc}', None

        if not self._started or self._client is None or self._node is None:
            return 503, 'runtime_unavailable', 'ROS bridge not started', None

        if not self.wait_for_server():
            return (
                503,
                'runtime_unavailable',
                f'action server {self.action_name} not available',
                None,
            )

        # Atomic: busy check + placeholder before any callback can fire.
        with self._lock:
            existing = self._live.get(mission_id)
            if existing is not None and not existing.terminal:
                return (
                    409,
                    'duplicate_mission_id',
                    f"mission_id '{mission_id}' already running",
                    None,
                )
            if self._busy_mission_id is not None:
                busy = self._busy_mission_id
                return (
                    409,
                    'busy',
                    f"runtime bridge busy with mission_id='{busy}'",
                    None,
                )
            placeholder = LiveMissionState(
                mission_id=mission_id,
                route_id=route_id,
                status='ACCEPTED',
                phase='VALIDATING',
                message='accepted by Runtime',
                observation_status=OBS_CLEAR,
            )
            self._live[mission_id] = placeholder
            self._busy_mission_id = mission_id
            self._result_attached.pop(mission_id, None)

        from inspection_interfaces.action import ExecuteInspection

        goal = ExecuteInspection.Goal()
        goal.mission_id = mission_id
        goal.route_id = route_id

        accept_box: dict[str, Any] = {'done': False, 'accepted': False, 'handle': None}
        accept_event = threading.Event()

        def _goal_response_cb(future) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:
                accept_box['done'] = True
                accept_box['accepted'] = False
                accept_box['comm_error'] = str(exc)
                accept_event.set()
                with self._lock:
                    st = self._live.get(mission_id)
                    late = st is not None and st.observation_status == OBS_PENDING
                    if late or (st is not None and not st.terminal):
                        # Communication error ≠ explicit reject.
                        self._mark_observation_unknown(
                            mission_id,
                            error_code='goal_response_communication_error',
                            message=f'goal response future failed (observation): {exc}',
                        )
                return
            accept_box['done'] = True
            accept_box['handle'] = goal_handle
            accept_box['accepted'] = bool(goal_handle and goal_handle.accepted)
            accept_event.set()
            with self._lock:
                st = self._live.get(mission_id)
                late = st is not None and st.observation_status == OBS_PENDING
            if accept_box['accepted'] and goal_handle is not None:
                # Late accept after HTTP 504: keep tracking and attach result once.
                with self._lock:
                    st = self._live.get(mission_id)
                    if st is not None and not st.terminal and late:
                        st.status = 'ACCEPTED'
                        st.phase = st.phase or 'VALIDATING'
                        st.message = 'late accept after observation PENDING; tracking'
                        st.observation_status = OBS_PENDING
                        self._busy_mission_id = mission_id
                self._attach_result_once(mission_id, route_id, goal_handle)
            elif late:
                # Late explicit reject: end unaccepted request and free busy.
                with self._lock:
                    self._finish_unaccepted_reject(
                        mission_id,
                        message='late Runtime reject after observation PENDING',
                    )

        def _feedback_cb(feedback_msg) -> None:
            fb = feedback_msg.feedback
            with self._lock:
                st = self._live.get(mission_id)
                if st is None or st.terminal:
                    return
                st.status = 'RUNNING'
                st.phase = str(fb.phase or st.phase)
                st.message = str(fb.message or '')
                if fb.station_id:
                    st.station_id = str(fb.station_id)
            try:
                self.console.set_current_goal({
                    'mission_id': mission_id,
                    'route_id': route_id,
                    'station_id': str(getattr(fb, 'station_id', '') or ''),
                    'phase': str(getattr(fb, 'phase', '') or ''),
                    'message': str(getattr(fb, 'message', '') or ''),
                })
            except Exception:
                pass

        try:
            send_future = self._client.send_goal_async(
                goal,
                feedback_callback=_feedback_cb,
            )
        except Exception as exc:
            with self._lock:
                self._mark_observation_unknown(
                    mission_id,
                    error_code='goal_send_communication_error',
                    message=f'send_goal_async failed (observation): {exc}',
                )
                snap = self.get_live(mission_id)
            return (
                503,
                'goal_send_communication_error',
                f'send_goal_async failed: {exc}',
                snap,
            )
        with self._lock:
            self._pending_send[mission_id] = send_future
        send_future.add_done_callback(_goal_response_cb)

        if not accept_event.wait(timeout=self.goal_response_timeout_sec):
            with self._lock:
                st = self._live.get(mission_id)
                # Keep tracking — do NOT invent FAILED terminal.
                if st is not None and not st.terminal:
                    st.observation_status = OBS_PENDING
                    st.error_code = 'goal_response_timeout'
                    st.message = (
                        'timed out waiting for Runtime goal accept/reject; '
                        'observation PENDING — continue polling, do not resubmit'
                    )
                    # Keep busy so we do not accept a second submit while unknown.
                    self._busy_mission_id = mission_id
            snap = self.get_live(mission_id)
            return (
                504,
                'goal_response_timeout',
                'timed out waiting for Runtime goal accept/reject',
                snap,
            )

        if not accept_box.get('accepted'):
            with self._lock:
                self._clear_busy_if(mission_id)
                st = self._live.get(mission_id)
                code = 'goal_rejected'
                msg = 'Runtime rejected ExecuteInspection goal (busy, duplicate, or unsafe)'
                if st is not None and st.terminal:
                    code = 'duplicate_mission_id'
                    msg = f"mission_id '{mission_id}' already used"
                elif st is not None and not st.terminal:
                    self._live.pop(mission_id, None)
            return 409, code, msg, None

        goal_handle = accept_box['handle']
        self._attach_result_once(mission_id, route_id, goal_handle)

        with self._lock:
            live = self._live.get(mission_id)
            if live is None:
                live = LiveMissionState(
                    mission_id=mission_id,
                    route_id=route_id,
                    status='ACCEPTED',
                    phase='VALIDATING',
                    message='accepted by Runtime',
                    observation_status=OBS_CLEAR,
                )
                self._live[mission_id] = live
            elif not live.terminal and live.status == 'ACCEPTED':
                live.message = live.message or 'accepted by Runtime'
                live.observation_status = OBS_CLEAR
            if not live.terminal:
                self._busy_mission_id = mission_id
            snapshot = LiveMissionState(**live.__dict__)

        return 202, 'NONE', 'accepted by Runtime', snapshot
