"""Async mission navigation runner for MVP-R1/R7.

Uses NavAdapter futures + done callbacks. Does not spin ROS from callbacks.

Supports:
- NavigateToPose (single pose) — A/B and transfers without vias
- NavigateThroughPoses (pose list) — e.g. B→C vias+goal in one Action

When defer_success=True (MVP-R2), nav SUCCEEDED signals arrival via
nav_arrived_event and does not publish COMPLETED — Runtime owns settle/capture
and the single terminal publish.

Timeout / stop rules (Fix B / V2-02):
- User cancel → external CANCELED only after Nav2 termination is confirmed.
- Send/result timeout → enter stop-confirm; keep mutex until Nav2 stop is
  confirmed or explicitly marked unsafe_busy. Never release solely because a
  cancel request was sent or a cancel response arrived.
- Late send after send-timeout still performs orphan cancel without a second
  terminal when a terminal was already published as unsafe.
- Normal result path releases only on confirmed Nav2 terminals
  (SUCCEEDED/ABORTED/CANCELED). UNKNOWN/EXECUTING/CANCELING/ACCEPTED are not
  terminals — enter stop-confirm / unsafe. on_finished runs outside runner lock
  via deferred flush (no manual RLock depth tricks).
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Optional  # BaseException is builtin

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

from inspection_mission.mission_state import (
    ErrorCode,
    ExecutionStatus,
    MissionRuntimeState,
    MissionTerminal,
    Phase,
    STATION_A,
)


FeedbackFn = Callable[[str, str, str, str], None]
FinishedFn = Callable[[MissionTerminal], None]

# Internal stop reasons (not Action fields).
STOP_USER_CANCEL = 'user_cancel'
STOP_SEND_TIMEOUT = 'send_timeout'
STOP_RESULT_TIMEOUT = 'result_timeout'

_TERMINAL_NAV_STATUSES = (
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
    GoalStatus.STATUS_SUCCEEDED,
)


class MissionRunner:
    """Drives one accepted ExecuteInspection goal through Nav2."""

    def __init__(
        self,
        *,
        state: MissionRuntimeState,
        nav: Any,
        mission_id: str,
        generation: int,
        pose: Optional[PoseStamped] = None,
        poses: Optional[list[PoseStamped]] = None,
        station_id: str = STATION_A,
        server_timeout_sec: float = 15.0,
        send_timeout_sec: float = 15.0,
        result_timeout_sec: float = 300.0,
        cancel_timeout_sec: float = 10.0,
        on_feedback: Optional[FeedbackFn] = None,
        on_finished: Optional[FinishedFn] = None,
        clock: Callable[[], float] = time.monotonic,
        defer_success: bool = False,
        nav_arrived_event: Optional[threading.Event] = None,
        managed_occupancy: bool = False,
    ) -> None:
        if poses is not None and len(poses) == 0:
            raise ValueError('poses must be non-empty when provided')
        if poses is None and pose is None:
            raise ValueError('pose or poses is required')
        self.state = state
        self.nav = nav
        self.poses = list(poses) if poses is not None else None
        self.use_through_poses = self.poses is not None
        self.pose = pose if pose is not None else self.poses[-1]  # type: ignore[index]
        self.mission_id = mission_id
        self.generation = generation
        self.station_id = station_id
        self.server_timeout_sec = server_timeout_sec
        self.send_timeout_sec = send_timeout_sec
        self.result_timeout_sec = result_timeout_sec
        self.cancel_timeout_sec = cancel_timeout_sec
        self.on_feedback = on_feedback or (lambda *_: None)
        self.on_finished = on_finished or (lambda *_: None)
        self._clock = clock
        self.defer_success = defer_success
        self.nav_arrived_event = nav_arrived_event or threading.Event()
        self.nav_arrived = False
        # Runtime-hosted: report leg result only; never release whole mission.
        self.managed_occupancy = bool(managed_occupancy)
        self._action_label = (
            'NavigateThroughPoses' if self.use_through_poses else 'NavigateToPose'
        )

        self._lock = threading.RLock()
        self._lock_depth = 0
        self._pending_finished: list[MissionTerminal] = []
        self._stop_mode: Optional[str] = None
        self._nav_goal_handle = None
        self._send_future = None
        self._result_future = None
        self._cancel_future = None
        self._send_deadline: Optional[float] = None
        self._result_deadline: Optional[float] = None
        self._cancel_deadline: Optional[float] = None
        self._started = False
        self._awaiting_send = False
        self._awaiting_result = False
        self._awaiting_cancel = False
        self._awaiting_unknown_send = False
        # Claimed before calling nav.send_*_async; cancel must not treat as "not sent".
        self._send_inflight_claimed = False
        self._local_terminal_done = False
        # After unsafe terminal, late send/result may still cancel Nav2.
        self._orphan_cleanup = False

    class _LockSection:
        def __init__(self, runner: 'MissionRunner') -> None:
            self._runner = runner

        def __enter__(self) -> 'MissionRunner._LockSection':
            self._runner._lock.acquire()
            self._runner._lock_depth += 1
            return self

        def __exit__(self, exc_type, exc, tb) -> None:
            self._runner._lock_depth -= 1
            self._runner._lock.release()
            if self._runner._lock_depth == 0:
                self._runner._flush_deferred_finished()

    def _section(self) -> 'MissionRunner._LockSection':
        return MissionRunner._LockSection(self)

    def _flush_deferred_finished(self) -> None:
        """Invoke on_finished outside runner lock. Safe to call anytime."""
        while True:
            with self._lock:
                if not self._pending_finished:
                    return
                batch = self._pending_finished
                self._pending_finished = []
            for terminal in batch:
                self.on_finished(terminal)

    def _still_owner(self) -> bool:
        # Safe under runner_lock → state.lock (state uses RLock).
        return self.state.is_owner(self.mission_id, self.generation)

    def _user_cancel_requested(self) -> bool:
        return self._stop_mode == STOP_USER_CANCEL

    def start(self) -> None:
        feedback_msgs: list[tuple[str, str]] = []
        with self._section():
            if self._started:
                return
            self._started = True
            if not self._still_owner():
                return
            self.state.set_phase(
                Phase.VALIDATING.value,
                mission_id=self.mission_id,
                generation=self.generation,
            )
            feedback_msgs.append((
                Phase.VALIDATING.value,
                f'validating station {self.station_id} navigation goal',
            ))
            if self._stop_mode == STOP_USER_CANCEL:
                # Will finish after releasing lock.
                pass

        for phase, msg in feedback_msgs:
            self._feedback(phase, msg)

        with self._section():
            cancel_before_nav = self._stop_mode == STOP_USER_CANCEL
            still = self._still_owner()
        if not still:
            return
        if cancel_before_nav:
            self._finish_canceled('canceled before navigation started')
            return

        # Preflight: wait_for_server is local; failure before send claim → releasable.
        if not self.nav.wait_for_server(
            self.server_timeout_sec, through_poses=self.use_through_poses,
        ):
            self._finish_failed(
                ErrorCode.NAV_SERVER_UNAVAILABLE,
                f'{self._action_label} action server unavailable',
                release=True,
            )
            return

        send_fn = None
        send_args = None
        nav_feedback = None
        with self._section():
            if not self._still_owner():
                return
            if self._stop_mode == STOP_USER_CANCEL:
                cancel_before_send = True
            else:
                cancel_before_send = False
                self.state.set_phase(
                    Phase.NAVIGATING.value,
                    mission_id=self.mission_id,
                    generation=self.generation,
                )
                if self.use_through_poses:
                    n = len(self.poses or [])
                    nav_feedback = (
                        Phase.NAVIGATING.value,
                        f'navigating through {n} poses to station {self.station_id}',
                    )
                    send_fn = self.nav.send_through_poses_async
                    send_args = (self.poses,)
                else:
                    nav_feedback = (
                        Phase.NAVIGATING.value,
                        f'navigating to station {self.station_id}',
                    )
                    send_fn = self.nav.send_goal_async
                    send_args = (self.pose,)
                # Claim before releasing lock / calling ROS send.
                self._send_inflight_claimed = True
                self._awaiting_send = True
                self._send_deadline = self._clock() + self.send_timeout_sec

        if cancel_before_send:
            self._finish_canceled(f'canceled before {self._action_label} send')
            return
        if nav_feedback is not None:
            self._feedback(*nav_feedback)

        # Lock outside: ROS send + callback registration (may run sync).
        send_future = None
        send_exc: Optional[BaseException] = None
        try:
            assert send_fn is not None and send_args is not None
            send_future = send_fn(
                *send_args, feedback_callback=self._on_nav_feedback,
            )
        except BaseException as exc:  # noqa: BLE001 — classify as control-unknown
            send_exc = exc

        with self._section():
            if not self._still_owner() and not self._orphan_cleanup:
                return
            if send_exc is not None:
                # Request may have left the process; cannot prove not sent.
                self._awaiting_send = False
                self._finish_control_unknown(
                    f'{self._action_label} send raised: {send_exc}',
                )
                return
            self._send_future = send_future
            fut = self._send_future

        if fut is not None:
            fut.add_done_callback(self._on_send_done)

    def request_cancel(self) -> None:
        handle = None
        orphan = False
        feedback = None
        soft_only = False
        with self._section():
            if self._orphan_cleanup:
                handle = self._nav_goal_handle
                if handle is not None and getattr(handle, 'accepted', False):
                    orphan = True
                else:
                    return
            elif self._local_terminal_done or not self._still_owner():
                return
            elif self.nav_arrived and self.defer_success:
                self._stop_mode = STOP_USER_CANCEL
                soft_only = True
                feedback = 'cancel requested after arrival; aborting capture'
            else:
                if self._stop_mode is None:
                    self._stop_mode = STOP_USER_CANCEL
                elif self._stop_mode in (STOP_SEND_TIMEOUT, STOP_RESULT_TIMEOUT):
                    self._stop_mode = STOP_USER_CANCEL
                feedback = 'cancel requested; confirming Nav2 stop'
                handle = self._nav_goal_handle
                if handle is None or not getattr(handle, 'accepted', False):
                    # Send claimed/in-flight or unknown — wait for _on_send_done.
                    handle = None
        if feedback is not None:
            self._enter_canceling_feedback(feedback)
        if soft_only:
            return
        if handle is not None:
            self._begin_cancel_confirm(handle, orphan=orphan)

    def mark_externally_finished(self) -> None:
        """Runtime published the single terminal after post-nav capture/cancel."""
        with self._section():
            self._local_terminal_done = True

    def user_cancel_requested(self) -> bool:
        return self._user_cancel_requested()

    def poll_timeouts(self) -> None:
        """Wall-clock send/result/cancel deadlines. Never release without stop confirm."""
        cancel_handle = None
        feedback = None
        with self._section():
            if self._local_terminal_done and not self._orphan_cleanup:
                return
            if not self._still_owner() and not self._orphan_cleanup:
                return
            now = self._clock()

            if self._awaiting_send and self._send_deadline is not None and now > self._send_deadline:
                self._awaiting_send = False
                self._send_deadline = None
                if self._stop_mode is None:
                    self._stop_mode = STOP_SEND_TIMEOUT
                feedback = (
                    'send acceptance timed out; holding mutex until Nav2 state known'
                )
                self._awaiting_unknown_send = True
                self._cancel_deadline = self._clock() + self.cancel_timeout_sec
            elif (
                self._awaiting_unknown_send
                and self._cancel_deadline is not None
                and now > self._cancel_deadline
            ):
                self._awaiting_unknown_send = False
                self._finish_failed(
                    ErrorCode.NAV_SEND_TIMEOUT,
                    f'send timed out and {self._action_label} acceptance never resolved',
                    release=False,
                    mark_unsafe=True,
                    allow_orphan_cleanup=True,
                )
                return
            elif (
                self._awaiting_result
                and self._result_deadline is not None
                and now > self._result_deadline
                and self._stop_mode is None
            ):
                self._awaiting_result = False
                self._result_deadline = None
                self._stop_mode = STOP_RESULT_TIMEOUT
                feedback = 'result timed out; canceling Nav2 and confirming stop'
                handle = self._nav_goal_handle
                if handle is not None and getattr(handle, 'accepted', False):
                    cancel_handle = handle
                else:
                    self._finish_stop_unconfirmed(
                        'result timed out without an accepted Nav2 goal handle',
                    )
                    return
            elif (
                self._awaiting_cancel
                and self._cancel_deadline is not None
                and now > self._cancel_deadline
            ):
                self._awaiting_cancel = False
                self._finish_stop_unconfirmed(
                    f'timed out confirming {self._action_label} stop',
                )
                return
            else:
                return

        if feedback is not None:
            self._enter_canceling_feedback(feedback)
        if cancel_handle is not None:
            self._begin_cancel_confirm(cancel_handle, orphan=False)

    def _enter_canceling_feedback(self, message: str) -> None:
        if self._local_terminal_done and not self._orphan_cleanup:
            return
        do_fb = False
        with self._section():
            if self._still_owner():
                self.state.set_phase(
                    Phase.CANCELING.value,
                    mission_id=self.mission_id,
                    generation=self.generation,
                )
                do_fb = True
        if do_fb:
            self._feedback(Phase.CANCELING.value, message)

    def _on_nav_feedback(self, _msg) -> None:
        return

    def _on_send_done(self, future) -> None:
        cancel_handle = None
        orphan_handle = None
        feedback_msg = None
        accepted_handle = None  # need get_result_async outside lock
        with self._section():
            # Orphan cleanup after unsafe send-timeout terminal.
            if self._local_terminal_done:
                if self._orphan_cleanup and self._still_owner():
                    orphan_handle = self._orphan_late_send_prepare_locked(future)
                else:
                    return
            elif not self._still_owner():
                return
            else:
                self._awaiting_send = False
                self._awaiting_unknown_send = False
                try:
                    handle = future.result()
                except Exception as exc:
                    # Future exception ≠ proof Nav2 idle (B1).
                    self._finish_control_unknown(
                        f'{self._action_label} send future failed: {exc}',
                    )
                    return

                self._nav_goal_handle = handle
                accepted = bool(getattr(handle, 'accepted', False))

                if self._stop_mode is not None:
                    if not accepted:
                        self._finish_after_stop_without_nav_goal(
                            f'{self._action_label} goal was not accepted during stop',
                        )
                        return
                    feedback_msg = (
                        'goal accepted during stop; confirming Nav2 termination'
                    )
                    cancel_handle = handle
                elif not accepted:
                    # Explicit reject — releasable.
                    self._finish_failed(
                        ErrorCode.NAV_GOAL_REJECTED,
                        f'{self._action_label} goal rejected by Nav2',
                        release=True,
                    )
                    return
                else:
                    accepted_handle = handle

        if feedback_msg is not None:
            self._enter_canceling_feedback(feedback_msg)
        if cancel_handle is not None:
            self._begin_cancel_confirm(cancel_handle, orphan=False)
        if orphan_handle is not None:
            self._begin_cancel_confirm(orphan_handle, orphan=True)
        if accepted_handle is None:
            return

        # ROS get_result + callback registration outside runner lock.
        result_exc: Optional[BaseException] = None
        result_future = None
        try:
            result_future = self.nav.get_result_async(accepted_handle)
        except BaseException as exc:  # noqa: BLE001
            result_exc = exc

        feedback_after = None
        cancel_after = None
        fut = None
        with self._section():
            if not self._still_owner():
                return
            if self._local_terminal_done:
                return
            if result_exc is not None:
                # Accepted but cannot attach result — attempt stop confirm.
                self._stop_mode = STOP_RESULT_TIMEOUT
                feedback_after = (
                    f'get_result_async after accept failed; confirming stop: '
                    f'{result_exc}'
                )
                cancel_after = accepted_handle
            else:
                self._result_future = result_future
                self._awaiting_result = True
                self._result_deadline = self._clock() + self.result_timeout_sec
                fut = result_future
        if feedback_after is not None:
            self._enter_canceling_feedback(feedback_after)
        if cancel_after is not None:
            self._begin_cancel_confirm(cancel_after, orphan=False)
            return
        if fut is not None:
            fut.add_done_callback(self._on_result_done)

    def _orphan_late_send_prepare_locked(self, future):
        """Late send after unsafe: return accepted handle for orphan cancel, or None."""
        self._awaiting_send = False
        self._awaiting_unknown_send = False
        try:
            handle = future.result()
        except Exception:
            return None
        self._nav_goal_handle = handle
        if not bool(getattr(handle, 'accepted', False)):
            return None
        return handle

    def _begin_cancel_confirm(self, handle, *, orphan: bool = False) -> None:
        """Claim cancel under lock; ROS cancel + callback registration outside."""
        with self._section():
            if self._awaiting_cancel:
                return
            if not orphan and not self._still_owner():
                return
            # Claim before lock-outside ROS call (send-inflight style).
            self._awaiting_cancel = True
            self._cancel_deadline = self._clock() + self.cancel_timeout_sec

        cancel_exc: Optional[BaseException] = None
        cancel_future = None
        try:
            cancel_future = self.nav.cancel_goal_async(handle)
        except BaseException as exc:  # noqa: BLE001
            cancel_exc = exc

        with self._section():
            if cancel_exc is not None:
                self._awaiting_cancel = False
                if orphan:
                    return
                if not self._still_owner():
                    return
                self._finish_stop_unconfirmed(f'cancel_goal_async failed: {cancel_exc}')
                return
            if not orphan and not self._still_owner() and not self._orphan_cleanup:
                # Cancel may already be in flight; keep tracking for late response.
                self._orphan_cleanup = True
            self._cancel_future = cancel_future
            fut = cancel_future
        if fut is not None:
            fut.add_done_callback(
                lambda f, orphan_flag=orphan: self._on_cancel_response_done(
                    f, orphan=orphan_flag,
                ),
            )

    def _on_cancel_response_done(self, future, *, orphan: bool = False) -> None:
        existing_result = None
        need_new_result = False
        handle_for_result = None
        call_stop_now = False
        with self._section():
            if self._local_terminal_done and not (orphan or self._orphan_cleanup):
                return
            if not orphan and not self._still_owner() and not self._orphan_cleanup:
                return
            try:
                response = future.result()
            except Exception as exc:
                self._awaiting_cancel = False
                if orphan or self._orphan_cleanup:
                    return
                self._finish_stop_unconfirmed(f'cancel response failed: {exc}')
                return

            # Cancel response alone is never proof the robot stopped.
            goals = getattr(response, 'goals_canceling', None)
            if goals is not None and len(goals) == 0:
                pass  # still wait for result

            handle = self._nav_goal_handle
            if handle is None:
                self._awaiting_cancel = False
                if orphan or self._orphan_cleanup:
                    return
                self._finish_stop_unconfirmed('missing Nav2 goal handle after cancel')
                return

            if self._result_future is None:
                need_new_result = True
                handle_for_result = handle
            else:
                existing_result = self._result_future
                if existing_result.done():
                    call_stop_now = True

        if call_stop_now and existing_result is not None:
            self._on_stop_result_done(existing_result, orphan=orphan)
            return
        if existing_result is not None:
            existing_result.add_done_callback(
                lambda fut, orphan_flag=orphan: self._on_stop_result_done(
                    fut, orphan=orphan_flag,
                ),
            )
            return
        if not need_new_result or handle_for_result is None:
            return

        # ROS get_result outside lock.
        result_exc: Optional[BaseException] = None
        result_future = None
        try:
            result_future = self.nav.get_result_async(handle_for_result)
        except BaseException as exc:  # noqa: BLE001
            result_exc = exc

        with self._section():
            if result_exc is not None:
                self._awaiting_cancel = False
                if orphan or self._orphan_cleanup:
                    return
                if not self._still_owner():
                    return
                self._finish_stop_unconfirmed(
                    f'get_result_async after cancel failed: {result_exc}',
                )
                return
            if self._local_terminal_done and not (orphan or self._orphan_cleanup):
                return
            if not orphan and not self._still_owner() and not self._orphan_cleanup:
                return
            self._result_future = result_future
            fut = result_future
        if fut is not None:
            fut.add_done_callback(
                lambda f, orphan_flag=orphan: self._on_stop_result_done(
                    f, orphan=orphan_flag,
                ),
            )

    def _on_stop_result_done(self, future, *, orphan: bool = False) -> None:
        with self._section():
            if self._local_terminal_done and not (orphan or self._orphan_cleanup):
                return
            if orphan or self._orphan_cleanup:
                # Cleanup only — swallow result, never publish another terminal.
                self._awaiting_cancel = False
                self._awaiting_result = False
                try:
                    future.result()
                except Exception:
                    pass
                return

            if not self._still_owner():
                return

            self._awaiting_cancel = False
            self._awaiting_result = False
            try:
                wrapped = future.result()
            except Exception as exc:
                self._finish_stop_unconfirmed(f'stop result failed: {exc}')
                return

            status = int(getattr(wrapped, 'status', GoalStatus.STATUS_UNKNOWN))
            if status in _TERMINAL_NAV_STATUSES:
                self._finish_after_nav_confirmed(status)
                return
            self._finish_stop_unconfirmed(
                f'Nav2 goal not confirmed terminated after stop (status={status})',
            )

    def _on_result_done(self, future) -> None:
        settle_msg = None
        route_stop = False
        route_orphan = False
        cancel_handle = None
        feedback_msg = None
        with self._section():
            if self._local_terminal_done:
                route_orphan = bool(self._orphan_cleanup)
            elif not self._still_owner():
                return
            elif self._stop_mode is not None:
                route_stop = True
            else:
                self._awaiting_result = False
                try:
                    wrapped = future.result()
                except Exception as exc:
                    handle = self._nav_goal_handle
                    if handle is not None and getattr(handle, 'accepted', False):
                        self._stop_mode = STOP_RESULT_TIMEOUT
                        feedback_msg = (
                            f'result future failed after accept; '
                            f'confirming Nav2 stop: {exc}'
                        )
                        cancel_handle = handle
                    else:
                        self._finish_control_unknown(
                            f'{self._action_label} result future failed: {exc}',
                        )
                else:
                    status = int(getattr(wrapped, 'status', GoalStatus.STATUS_UNKNOWN))
                    if status == GoalStatus.STATUS_SUCCEEDED:
                        if self.defer_success:
                            self.nav_arrived = True
                            settle_msg = (
                                f'arrived at station {self.station_id}; '
                                'handing off to settle/capture'
                            )
                        else:
                            self._finish_completed(
                                f'{self.station_id}-station navigation Action completed '
                                '(navigation sub-flow only; not acceptance/vision/archive)',
                            )
                    elif status == GoalStatus.STATUS_ABORTED:
                        self._finish_failed(
                            ErrorCode.NAV_FAILED,
                            f'{self._action_label} aborted (status={status})',
                            release=True,
                        )
                    elif status == GoalStatus.STATUS_CANCELED:
                        self._finish_canceled(
                            f'{self._action_label} canceled by Nav2 (status={status})',
                        )
                    else:
                        # UNKNOWN / ACCEPTED / EXECUTING / CANCELING — not terminals.
                        self._stop_mode = STOP_RESULT_TIMEOUT
                        feedback_msg = (
                            f'non-terminal Nav2 result status={status}; '
                            'confirming stop before release'
                        )
                        handle = self._nav_goal_handle
                        if handle is not None and getattr(handle, 'accepted', False):
                            cancel_handle = handle
                        else:
                            self._finish_stop_unconfirmed(
                                f'non-terminal Nav2 status={status} '
                                'without accepted handle',
                            )

        if settle_msg is not None:
            self._feedback(Phase.SETTLING.value, settle_msg)
            self.nav_arrived_event.set()
            return
        if feedback_msg is not None:
            self._enter_canceling_feedback(feedback_msg)
        if cancel_handle is not None:
            self._begin_cancel_confirm(cancel_handle, orphan=False)
            return
        if route_orphan:
            self._on_stop_result_done(future, orphan=True)
        elif route_stop:
            self._on_stop_result_done(future, orphan=False)

    def _feedback(self, phase: str, message: str) -> None:
        if not self._still_owner():
            return
        self.on_feedback(self.mission_id, self.station_id, phase, message)

    def _finish_after_stop_without_nav_goal(self, message: str) -> None:
        if self._stop_mode == STOP_USER_CANCEL:
            self._finish_canceled(message)
        elif self._stop_mode == STOP_SEND_TIMEOUT:
            self._finish_failed(ErrorCode.NAV_SEND_TIMEOUT, message, release=True)
        elif self._stop_mode == STOP_RESULT_TIMEOUT:
            self._finish_failed(ErrorCode.NAV_RESULT_TIMEOUT, message, release=True)
        else:
            self._finish_canceled(message)

    def _finish_after_nav_confirmed(self, nav_status: int) -> None:
        msg = f'Nav2 stopped with status={nav_status}'
        if self._stop_mode == STOP_USER_CANCEL:
            self._finish_canceled(f'A-station navigation canceled ({msg})')
        elif self._stop_mode == STOP_SEND_TIMEOUT:
            self._finish_failed(
                ErrorCode.NAV_SEND_TIMEOUT,
                f'send timed out; underlying goal confirmed stopped ({msg})',
                release=True,
            )
        elif self._stop_mode == STOP_RESULT_TIMEOUT:
            self._finish_failed(
                ErrorCode.NAV_RESULT_TIMEOUT,
                f'result timed out; underlying goal confirmed stopped ({msg})',
                release=True,
            )
        else:
            self._finish_canceled(f'A-station navigation canceled ({msg})')

    def _finish_stop_unconfirmed(self, message: str) -> None:
        # Prefer cancel_unconfirmed when stop confirmation failed.
        # Preserve original timeout code in message; error_code stays cancel_unconfirmed
        # when we cannot prove termination (spec: FAILED + cancel_unconfirmed).
        code = ErrorCode.CANCEL_UNCONFIRMED
        if self._stop_mode == STOP_SEND_TIMEOUT:
            message = f'{ErrorCode.NAV_SEND_TIMEOUT}: {message}'
        elif self._stop_mode == STOP_RESULT_TIMEOUT:
            message = f'{ErrorCode.NAV_RESULT_TIMEOUT}: {message}'
        self._finish_failed(
            code,
            message,
            release=False,
            mark_unsafe=True,
            allow_orphan_cleanup=True,
        )

    def _finish_control_unknown(self, message: str) -> None:
        """Nav request may be in flight; keep occupancy (B1)."""
        self._finish_failed(
            ErrorCode.CANCEL_UNCONFIRMED,
            message,
            release=False,
            mark_unsafe=True,
            allow_orphan_cleanup=True,
        )

    def _finish_completed(self, message: str) -> None:
        self._emit_terminal(
            ExecutionStatus.COMPLETED.value,
            ErrorCode.NONE,
            message,
            release=True,
        )

    def _finish_failed(
        self,
        error_code: str,
        message: str,
        *,
        release: bool,
        mark_unsafe: bool = False,
        allow_orphan_cleanup: bool = False,
    ) -> None:
        self._emit_terminal(
            ExecutionStatus.FAILED.value,
            error_code,
            message,
            release=release,
            mark_unsafe=mark_unsafe,
            allow_orphan_cleanup=allow_orphan_cleanup,
        )

    def _finish_canceled(self, message: str) -> None:
        self._emit_terminal(
            ExecutionStatus.CANCELED.value,
            ErrorCode.NONE,
            message,
            release=True,
        )

    def _emit_terminal(
        self,
        execution_status: str,
        error_code: str,
        message: str,
        *,
        release: bool,
        mark_unsafe: bool = False,
        allow_orphan_cleanup: bool = False,
    ) -> None:
        """Claim at most one terminal; defer on_finished to lock-section exit.

        managed_occupancy=True (Runtime-hosted): report leg result only — never
        release/mark whole-mission occupancy via try_publish_terminal.
        """

        def _claim() -> None:
            if self._local_terminal_done:
                return
            if self.managed_occupancy:
                if not self._still_owner():
                    return
                terminal = MissionTerminal(
                    mission_id=self.mission_id,
                    execution_status=execution_status,
                    error_code=error_code,
                    message=message,
                    generation=self.generation,
                    station_id=self.station_id,
                )
                # Optionally mark unsafe on state without releasing occupancy.
                if mark_unsafe:
                    with self.state.lock:
                        if self.state.is_owner_unlocked(self.mission_id, self.generation):
                            self.state.unsafe_busy = True
                            self.state.phase = Phase.CANCELING.value
                self._local_terminal_done = True
                self._orphan_cleanup = bool(allow_orphan_cleanup and mark_unsafe)
                self._pending_finished.append(terminal)
                return
            terminal = self.state.try_publish_terminal(
                self.mission_id,
                execution_status,
                error_code,
                message,
                release=release,
                mark_unsafe=mark_unsafe,
                generation=self.generation,
            )
            if terminal is None:
                return
            self._local_terminal_done = True
            self._orphan_cleanup = bool(allow_orphan_cleanup and mark_unsafe)
            self._pending_finished.append(terminal)

        if self._lock_depth > 0:
            _claim()
            return
        with self._section():
            _claim()
