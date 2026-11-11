"""Process-local mission mutex, dedup, ownership, and terminal-once guards.

V2-02B1: authoritative state RLock; Window/Slot use unlocked helpers under that
lock only (no nested plain Lock). Lock order with MissionRunner: runner→state.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional, Tuple

from inspection_mission.route_plan import (  # noqa: E402
    ALLOWED_ROUTE_ID,
    ALLOWED_ROUTE_IDS,
    ROUTE_A,
    ROUTE_ABC,
    STATION_A,
    aggregate_inspection_status,
    stations_for_route,
)


class ExecutionStatus(str, Enum):
    COMPLETED = 'COMPLETED'
    FAILED = 'FAILED'
    CANCELED = 'CANCELED'
    INTERRUPTED = 'INTERRUPTED'


class Phase(str, Enum):
    VALIDATING = 'VALIDATING'
    NAVIGATING = 'NAVIGATING'
    CANCELING = 'CANCELING'
    SETTLING = 'SETTLING'
    CAPTURING = 'CAPTURING'
    INSPECTING = 'INSPECTING'


class ErrorCode:
    NONE = 'NONE'
    EMPTY_MISSION_ID = 'empty_mission_id'
    UNSUPPORTED_ROUTE = 'unsupported_route'
    BUSY = 'busy'
    DUPLICATE_MISSION_ID = 'duplicate_mission_id'
    RUNTIME_UNSAFE = 'runtime_unsafe_busy'
    SAFETY_REFUSE = 'safety_refuse'
    NAV_SERVER_UNAVAILABLE = 'nav_server_unavailable'
    NAV_GOAL_REJECTED = 'nav_goal_rejected'
    NAV_FAILED = 'nav_failed'
    NAV_SEND_TIMEOUT = 'nav_send_timeout'
    NAV_RESULT_TIMEOUT = 'nav_result_timeout'
    CANCEL_UNCONFIRMED = 'cancel_unconfirmed'
    SETTLE_TIMEOUT = 'settle_timeout'
    ODOM_STALE = 'odom_stale'
    NO_NEW_FRAME = 'no_new_frame'
    IMAGE_CONVERT_FAILED = 'image_convert_failed'
    IMAGE_SAVE_FAILED = 'image_save_failed'
    VISION_FAILED = 'vision_failed'
    ARCHIVE_FAILED = 'archive_failed'
    INTERNAL_ERROR = 'internal_error'


# Independent HTTP/bridge observation (not Action execution_status).
class ObservationStatus:
    CLEAR = 'CLEAR'
    PENDING = 'PENDING'
    UNKNOWN = 'UNKNOWN'


@dataclass
class MissionTerminal:
    mission_id: str
    execution_status: str
    error_code: str
    message: str
    generation: Optional[int] = None
    screenshot_path: str = ''
    image_stamp: str = ''
    frame_id: str = ''
    station_id: str = ''
    inspection_status: str = ''
    station_result_path: str = ''
    model_version: str = ''


@dataclass
class MissionContext:
    """Per-accepted-goal isolation for execute handoff (not shared across missions)."""

    mission_id: str
    generation: int
    goal_handle: Any = None
    runner: Any = None
    finish_event: threading.Event = field(default_factory=threading.Event)
    finish_terminal: Optional[MissionTerminal] = None
    pending_cancel: bool = False
    nav_arrived_event: threading.Event = field(default_factory=threading.Event)
    started_at: str = ''
    route_id: str = ''

    def publish_finished(self, terminal: MissionTerminal) -> None:
        """Record this context's terminal and wake only this context's waiter."""
        self.finish_terminal = terminal
        self.finish_event.set()


class MissionActiveSlot:
    """Runtime-owned pointer to the current MissionContext.

    Public methods take the shared state RLock. Unlocked helpers may only be
    called while that lock is already held.
    """

    def __init__(self, lock: Optional[threading.RLock] = None) -> None:
        self._lock = lock if lock is not None else threading.RLock()
        self._active: Optional[MissionContext] = None
        self._contexts: dict[int, MissionContext] = {}

    def bind_lock(self, lock: threading.RLock) -> None:
        self._lock = lock

    @property
    def active(self) -> Optional[MissionContext]:
        with self._lock:
            return self._active

    def _key(self, ctx: MissionContext) -> int:
        if ctx.goal_handle is not None:
            return id(ctx.goal_handle)
        return id(ctx)

    def install_unlocked(self, ctx: MissionContext) -> None:
        self._contexts[self._key(ctx)] = ctx
        self._active = ctx

    def get_by_goal_handle_unlocked(
        self, goal_handle: Any,
    ) -> Optional[MissionContext]:
        ctx = self._contexts.get(id(goal_handle))
        if ctx is not None:
            return ctx
        if self._active is not None and self._active.goal_handle is goal_handle:
            return self._active
        return None

    def clear_if_owner_unlocked(self, ctx: MissionContext) -> bool:
        self._contexts.pop(self._key(ctx), None)
        if self._active is ctx:
            self._active = None
            return True
        return False

    def install(self, ctx: MissionContext) -> None:
        with self._lock:
            self.install_unlocked(ctx)

    def get_by_goal_handle(self, goal_handle: Any) -> Optional[MissionContext]:
        with self._lock:
            return self.get_by_goal_handle_unlocked(goal_handle)

    def clear_if_owner(self, ctx: MissionContext) -> bool:
        with self._lock:
            return self.clear_if_owner_unlocked(ctx)


@dataclass
class AcceptedGoalTicket:
    """Reservation for the ACCEPT → MissionContext-install window."""

    mission_id: str
    generation: int
    pending_cancel: bool = False
    goal_handle: Any = None


class AcceptedGoalWindow:
    """Bridge cancels that arrive after ACCEPT but before Context install.

    No private Lock — callers must hold MissionRuntimeState.lock (or pass through
    Bridge which does). Public wrappers acquire that shared lock once.
    """

    def __init__(self, lock: Optional[threading.RLock] = None) -> None:
        self._lock = lock if lock is not None else threading.RLock()
        self._ticket: Optional[AcceptedGoalTicket] = None

    def bind_lock(self, lock: threading.RLock) -> None:
        self._lock = lock

    @property
    def ticket(self) -> Optional[AcceptedGoalTicket]:
        with self._lock:
            return self._ticket

    def open_unlocked(self, mission_id: str, generation: int) -> None:
        self._ticket = AcceptedGoalTicket(
            mission_id=mission_id,
            generation=generation,
        )

    def clear_unlocked(self) -> None:
        self._ticket = None

    def record_cancel_unlocked(
        self,
        goal_handle: Any,
        state: 'MissionRuntimeState',
    ) -> bool:
        """Accept cancel for awaiting ticket. Caller holds state.lock."""
        ticket = self._ticket
        if ticket is None:
            return False
        request = getattr(goal_handle, 'request', None)
        mission_id = str(getattr(request, 'mission_id', '') or '').strip()
        if mission_id != ticket.mission_id:
            return False
        if not state.is_owner_unlocked(mission_id, ticket.generation):
            return False
        if ticket.goal_handle is None:
            ticket.goal_handle = goal_handle
        elif ticket.goal_handle is not goal_handle:
            return False
        ticket.pending_cancel = True
        return True

    def consume_unlocked(
        self,
        goal_handle: Any,
        mission_id: str,
        generation: int,
    ) -> bool:
        ticket = self._ticket
        self._ticket = None
        if ticket is None:
            return False
        if ticket.mission_id != mission_id or ticket.generation != generation:
            return False
        if ticket.goal_handle is not None and ticket.goal_handle is not goal_handle:
            return False
        return bool(ticket.pending_cancel)

    def open(self, mission_id: str, generation: int) -> None:
        with self._lock:
            self.open_unlocked(mission_id, generation)

    def clear(self) -> None:
        with self._lock:
            self.clear_unlocked()

    def record_cancel_for_awaiting(
        self,
        goal_handle: Any,
        state: 'MissionRuntimeState',
    ) -> bool:
        with self._lock:
            return self.record_cancel_unlocked(goal_handle, state)

    def consume_for_install(
        self,
        goal_handle: Any,
        mission_id: str,
        generation: int,
    ) -> bool:
        with self._lock:
            return self.consume_unlocked(goal_handle, mission_id, generation)


class AcceptCancelBridge:
    """Production cancel resolution used by MissionRuntimeNode callbacks.

    Cancel: record under state.lock; request_cancel outside.
    Install: consume ticket + slot.install in one critical section.
    """

    def __init__(
        self,
        state: 'MissionRuntimeState',
        slot: MissionActiveSlot,
        window: Optional[AcceptedGoalWindow] = None,
    ) -> None:
        self.state = state
        self.slot = slot
        self.slot.bind_lock(state.lock)
        self.window = window or AcceptedGoalWindow(lock=state.lock)
        self.window.bind_lock(state.lock)

    def on_goal_accepted(self, mission_id: str, generation: int) -> None:
        # Prefer atomic reserve_goal; this remains for callers that already began.
        with self.state.lock:
            self.window.open_unlocked(mission_id, generation)

    def on_goal_reject_cleanup(self) -> None:
        with self.state.lock:
            self.window.clear_unlocked()

    def resolve_cancel(self, goal_handle: Any) -> bool:
        """Return True if cancel is ACCEPT (recorded / forwarded), else REJECT."""
        runner = None
        accepted = False
        with self.state.lock:
            ctx = self.slot.get_by_goal_handle_unlocked(goal_handle)
            if ctx is not None and ctx.goal_handle is goal_handle:
                ctx.pending_cancel = True
                runner = ctx.runner
                accepted = True
            elif self.window.record_cancel_unlocked(goal_handle, self.state):
                accepted = True
        # Lock outside: ROS / runner side effects.
        if runner is not None:
            runner.request_cancel()
        return accepted

    def install_context_after_accepted(
        self,
        goal_handle: Any,
        mission_id: str,
        generation: int,
    ) -> MissionContext:
        with self.state.lock:
            pending_cancel = self.window.consume_unlocked(
                goal_handle, mission_id, generation,
            )
            ctx = MissionContext(
                mission_id=mission_id,
                generation=generation,
                goal_handle=goal_handle,
                pending_cancel=pending_cancel,
            )
            self.slot.install_unlocked(ctx)
            return ctx


@dataclass
class MissionRuntimeState:
    """Single active mission + process-local used mission_id set + owner generation."""

    active_mission_id: Optional[str] = None
    active_generation: Optional[int] = None
    phase: Optional[str] = None
    used_mission_ids: set[str] = field(default_factory=set)
    terminal_published: bool = False
    # Cancel confirmation failed / underlying nav state unknown.
    unsafe_busy: bool = False
    # Startup / clear-failure refuse (safety mark). Distinct message from unsafe_busy.
    safety_refuse: bool = False
    safety_refuse_reason: str = ''
    # Claimed candidate terminal not yet archive-committed / Action-published.
    finalizing: bool = False
    archive_degraded: bool = False
    recovery_claimed: bool = False
    last_terminal: Optional[MissionTerminal] = None
    _next_generation: int = 0

    def __post_init__(self) -> None:
        self._lock = threading.RLock()

    @property
    def lock(self) -> threading.RLock:
        return self._lock

    @property
    def is_busy(self) -> bool:
        with self._lock:
            return self._is_busy_unlocked()

    def _is_busy_unlocked(self) -> bool:
        return (
            self.unsafe_busy
            or self.safety_refuse
            or self.archive_degraded
            or self.active_mission_id is not None
            or bool(getattr(self, 'recovery_claimed', False))
        )

    def set_safety_refuse(self, refuse: bool, reason: str = '') -> None:
        with self._lock:
            self.safety_refuse = bool(refuse)
            self.safety_refuse_reason = str(reason or '')

    def clear_safety_refuse_unlocked(self) -> None:
        self.safety_refuse = False
        self.safety_refuse_reason = ''

    def _validate_goal_unlocked(
        self, mission_id: str, route_id: str,
    ) -> tuple[bool, str, str]:
        if mission_id is None or not str(mission_id).strip():
            return False, ErrorCode.EMPTY_MISSION_ID, 'mission_id must be non-empty'
        mission_id = str(mission_id).strip()
        if str(route_id).strip() not in ALLOWED_ROUTE_IDS:
            return (
                False,
                ErrorCode.UNSUPPORTED_ROUTE,
                f"route_id must be one of {sorted(ALLOWED_ROUTE_IDS)}, got '{route_id}'",
            )
        if self.safety_refuse:
            return (
                False,
                ErrorCode.SAFETY_REFUSE,
                self.safety_refuse_reason
                or 'robot safety mark refuses new goals until operator confirm',
            )
        if self.archive_degraded:
            return (
                False,
                ErrorCode.ARCHIVE_FAILED,
                'archive_degraded: refusing new goals until operator confirm',
            )
        if self.unsafe_busy:
            return (
                False,
                ErrorCode.RUNTIME_UNSAFE,
                'runtime is unsafe/busy after unconfirmed cancel; refusing new goals',
            )
        if getattr(self, 'recovery_claimed', False):
            return (
                False,
                ErrorCode.BUSY,
                'recovery in progress; refusing new goals until confirm finishes',
            )
        if self.active_mission_id is not None:
            return (
                False,
                ErrorCode.BUSY,
                f"runtime busy with mission_id='{self.active_mission_id}' phase={self.phase}",
            )
        if mission_id in self.used_mission_ids:
            return (
                False,
                ErrorCode.DUPLICATE_MISSION_ID,
                f"mission_id '{mission_id}' already used "
                f'(process memory and/or SQLite archive)',
            )
        return True, ErrorCode.NONE, ''

    def validate_goal(self, mission_id: str, route_id: str) -> tuple[bool, str, str]:
        """Return (ok, error_code, message). Does not mutate state."""
        with self._lock:
            return self._validate_goal_unlocked(mission_id, route_id)

    def seed_used_mission_ids(self, mission_ids: set[str]) -> None:
        """Load persisted mission_ids so restarts cannot silently reuse them."""
        with self._lock:
            self.used_mission_ids.update(mission_ids)

    def _begin_unlocked(self, mission_id: str) -> int:
        if self._is_busy_unlocked():
            raise RuntimeError('cannot begin mission while busy/unsafe')
        self._next_generation += 1
        generation = self._next_generation
        self.active_mission_id = mission_id
        self.active_generation = generation
        self.phase = Phase.VALIDATING.value
        self.terminal_published = False
        self.finalizing = False
        self.archive_degraded = False
        self.recovery_claimed = False
        self.last_terminal = None
        return generation

    def begin(self, mission_id: str) -> int:
        """Reserve the active slot. Returns the new owner generation."""
        with self._lock:
            return self._begin_unlocked(mission_id)

    def reserve_goal(
        self,
        mission_id: str,
        route_id: str,
        window: AcceptedGoalWindow,
    ) -> Tuple[bool, str, str, Optional[int]]:
        """Atomic validate + begin + ticket.open. Returns (ok, code, msg, generation)."""
        mission_id = str(mission_id or '').strip()
        with self._lock:
            ok, code, msg = self._validate_goal_unlocked(mission_id, route_id)
            if not ok:
                return False, code, msg, None
            generation = self._begin_unlocked(mission_id)
            window.open_unlocked(mission_id, generation)
            return True, ErrorCode.NONE, '', generation

    def is_owner_unlocked(self, mission_id: str, generation: int) -> bool:
        return (
            self.active_mission_id == mission_id
            and self.active_generation == generation
        )

    def is_owner(self, mission_id: str, generation: int) -> bool:
        with self._lock:
            return self.is_owner_unlocked(mission_id, generation)

    def set_phase(
        self,
        phase: str,
        *,
        mission_id: str,
        generation: int,
    ) -> bool:
        """Update phase only for the current owner. Returns False if rejected."""
        with self._lock:
            if not self.is_owner_unlocked(mission_id, generation):
                return False
            self.phase = phase
            return True

    def try_publish_terminal(
        self,
        mission_id: str,
        execution_status: str,
        error_code: str,
        message: str,
        *,
        release: bool,
        mark_unsafe: bool = False,
        generation: Optional[int] = None,
        screenshot_path: str = '',
        image_stamp: str = '',
        frame_id: str = '',
        station_id: str = '',
        inspection_status: str = '',
        station_result_path: str = '',
        model_version: str = '',
    ) -> Optional[MissionTerminal]:
        """Publish at most one terminal for the owning generation.

        Returns the terminal when this call owns the first publish; otherwise None.
        Never clears unsafe_busy once set (B1: no auto-clear).

        For Runtime archive linearization prefer claim_finalizing_terminal +
        commit_terminal_outcome so occupancy is not released before archive commit.
        """
        with self._lock:
            return self._publish_terminal_unlocked(
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
                finalizing=False,
            )

    def claim_finalizing_terminal(
        self,
        mission_id: str,
        execution_status: str,
        error_code: str,
        message: str,
        *,
        generation: int,
        mark_unsafe: bool = False,
        screenshot_path: str = '',
        image_stamp: str = '',
        frame_id: str = '',
        station_id: str = '',
        inspection_status: str = '',
        station_result_path: str = '',
        model_version: str = '',
    ) -> Optional[MissionTerminal]:
        """Claim candidate terminal without releasing occupancy (archive pending)."""
        with self._lock:
            return self._publish_terminal_unlocked(
                mission_id,
                execution_status,
                error_code,
                message,
                release=False,
                mark_unsafe=mark_unsafe,
                generation=generation,
                screenshot_path=screenshot_path,
                image_stamp=image_stamp,
                frame_id=frame_id,
                station_id=station_id,
                inspection_status=inspection_status,
                station_result_path=station_result_path,
                model_version=model_version,
                finalizing=True,
            )

    def commit_archive_result(
        self,
        mission_id: str,
        generation: int,
        *,
        archive_ok: bool,
        mark_unsafe: bool = False,
        override_terminal: Optional[MissionTerminal] = None,
    ) -> Optional[MissionTerminal]:
        """Apply archive outcome while KEEPING occupancy (R2).

        Never releases. Old/non-owner calls must not rewrite last_terminal.
        """
        with self._lock:
            if not self.is_owner_unlocked(mission_id, generation):
                return None
            if override_terminal is not None:
                self.last_terminal = override_terminal
            terminal = self.last_terminal
            self.finalizing = False
            if not archive_ok:
                self.archive_degraded = True
                self.unsafe_busy = True
                self.phase = Phase.CANCELING.value
                return terminal
            self.archive_degraded = False
            if mark_unsafe:
                self.unsafe_busy = True
                self.phase = Phase.CANCELING.value
            return terminal

    def release_occupancy_if_owner(
        self,
        mission_id: str,
        generation: int,
        *,
        mark_unsafe: bool = False,
    ) -> bool:
        """Atomic open for new tasks only while still owner/generation."""
        with self._lock:
            if not self.is_owner_unlocked(mission_id, generation):
                return False
            self.finalizing = False
            if mark_unsafe:
                self.unsafe_busy = True
                self.phase = Phase.CANCELING.value
                return True
            self.active_mission_id = None
            self.active_generation = None
            self.phase = None
            return True

    def commit_terminal_outcome(
        self,
        mission_id: str,
        generation: int,
        *,
        archive_ok: bool,
        release: bool,
        mark_unsafe: bool = False,
        override_terminal: Optional[MissionTerminal] = None,
    ) -> Optional[MissionTerminal]:
        """Compatibility wrapper: archive result, optionally release if still owner."""
        terminal = self.commit_archive_result(
            mission_id,
            generation,
            archive_ok=archive_ok,
            mark_unsafe=mark_unsafe,
            override_terminal=override_terminal,
        )
        if terminal is None:
            return None
        if archive_ok and release and not mark_unsafe:
            self.release_occupancy_if_owner(
                mission_id, generation, mark_unsafe=False,
            )
        return terminal

    def can_accept_operator_confirm(self) -> tuple[bool, str]:
        """Refuse confirm while mission running / finalizing / recovering."""
        with self._lock:
            if self.finalizing:
                return False, 'finalizing in progress; refuse confirm'
            if self.active_mission_id is not None and not self.terminal_published:
                return False, 'active mission still executing; refuse confirm'
            if getattr(self, 'recovery_claimed', False):
                return False, 'recovery already claimed'
            return True, ''

    def claim_recovery(self) -> bool:
        with self._lock:
            ok, _ = self.can_accept_operator_confirm()
            if not ok:
                # Still allow when only barriers (no active mission).
                if self.finalizing:
                    return False
                if self.active_mission_id is not None and not self.terminal_published:
                    return False
                if getattr(self, 'recovery_claimed', False):
                    return False
            self.recovery_claimed = True
            return True

    def clear_recovery_claim(self) -> None:
        with self._lock:
            self.recovery_claimed = False

    def operator_clear_barriers(self) -> None:
        """After interrupt+clear success: drop refuse barriers if idle."""
        with self._lock:
            if self.finalizing:
                return
            if self.active_mission_id is not None and not self.terminal_published:
                return
            self.safety_refuse = False
            self.safety_refuse_reason = ''
            self.archive_degraded = False
            self.unsafe_busy = False
            self.recovery_claimed = False
            if self.terminal_published or self.active_mission_id is None:
                self.active_mission_id = None
                self.active_generation = None
                self.phase = None
                self.terminal_published = False

    def _publish_terminal_unlocked(
        self,
        mission_id: str,
        execution_status: str,
        error_code: str,
        message: str,
        *,
        release: bool,
        mark_unsafe: bool,
        generation: Optional[int],
        screenshot_path: str,
        image_stamp: str,
        frame_id: str,
        station_id: str,
        inspection_status: str,
        station_result_path: str,
        model_version: str,
        finalizing: bool,
    ) -> Optional[MissionTerminal]:
        if generation is None or not self.is_owner_unlocked(mission_id, generation):
            return None
        if self.terminal_published:
            return None
        self.terminal_published = True
        self.finalizing = bool(finalizing)
        terminal = MissionTerminal(
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
        self.last_terminal = terminal
        self.used_mission_ids.add(mission_id)
        if mark_unsafe:
            self.unsafe_busy = True
            self.active_mission_id = mission_id
            self.active_generation = generation
            self.phase = Phase.CANCELING.value
        elif release and not finalizing:
            self.active_mission_id = None
            self.active_generation = None
            self.phase = None
        return terminal
