"""Pure mission-timing state machine, with no ROS dependency.

Kept separate from mission_clock_node.py for the same reason path_planner.py
and zone_splitter.py are separate from the executor that calls them: the
timing rules (when does the clock start, when does it freeze, what counts as
a new run) are worth unit-testing directly, without standing up a ROS graph
or a simulator to do it.

The clock is driven purely by the mission state names nidar_mission_executor
already publishes on /nidar/mission_status -- it does not observe drones,
flight, or anything else. One task: measure how long a mission run takes.
"""

from __future__ import annotations

import time
from typing import Callable, List, Optional, Tuple

# Mission states published by nidar_mission_executor._publish_status. Kept as
# plain strings (not an enum shared with the executor) deliberately: this node
# consumes the executor's *published wire format*, so it should tolerate an
# unrecognised state by treating it as "some in-flight phase" rather than
# crashing -- see on_state.
STATE_IDLE = 'idle'
STATE_LOADED = 'loaded'
STATE_COMPLETE = 'complete'
STATE_ERROR = 'error'

#: States that mean "no mission run is in progress and none has been asked
#: for yet". Reaching one of these never starts the clock.
PRE_RUN_STATES = frozenset({STATE_IDLE, STATE_LOADED})

#: States that end a run. The elapsed time freezes here and stays readable
#: (that final total is the number the operator actually wants) until a new
#: run starts.
TERMINAL_STATES = frozenset({STATE_COMPLETE, STATE_ERROR})


class MissionClock:
    """Tracks elapsed wall-clock time for one mission run, plus a
    per-phase breakdown of where that time went.

    Uses a monotonic clock, not wall time: a mission is a *duration*
    measurement, and time.time() can jump backwards (NTP correction, manual
    clock change) mid-run and produce a negative or wildly wrong total.
    """

    def __init__(self, now_fn: Callable[[], float] = time.monotonic):
        self._now_fn = now_fn
        self._start: Optional[float] = None
        self._frozen_elapsed: float = 0.0
        self._running: bool = False
        self._phase: Optional[str] = None
        self._phase_started: float = 0.0
        self._phases: List[Tuple[str, float]] = []
        self._last_state: str = STATE_IDLE

    # -- queries -------------------------------------------------------

    @property
    def running(self) -> bool:
        return self._running

    @property
    def state(self) -> str:
        """The last mission state this clock was told about."""
        return self._last_state

    @property
    def phase(self) -> Optional[str]:
        """The phase currently being timed, or None outside a run."""
        return self._phase

    def elapsed(self) -> float:
        """Seconds since this run started -- live while running, frozen at
        the final total once the run has ended, 0.0 before any run."""
        if self._running and self._start is not None:
            return self._now_fn() - self._start
        return self._frozen_elapsed

    def phase_durations(self) -> List[Tuple[str, float]]:
        """Completed phases in order, as (state_name, seconds). The phase
        still in progress is included with its time so far, so a breakdown
        requested mid-run still adds up to elapsed()."""
        done = list(self._phases)
        if self._running and self._phase is not None:
            done.append((self._phase, self._now_fn() - self._phase_started))
        return done

    # -- driving -------------------------------------------------------

    def on_state(self, state: str) -> None:
        """Feed one mission state transition in.

        Repeated identical states are ignored (the executor republishes
        'running' once per waypoint in the JSON-waypoint flow, and those
        are progress updates within one phase, not phase changes).
        """
        now = self._now_fn()
        self._last_state = state

        if state in PRE_RUN_STATES:
            # 'loaded' after a finished run deliberately does NOT clear the
            # previous total -- the operator is setting up the next mission
            # and the last run's time should stay on screen until the new
            # one actually starts. A fresh run resets it below.
            return

        if state in TERMINAL_STATES:
            if self._running:
                self._close_phase(now)
                self._frozen_elapsed = now - (self._start or now)
                self._running = False
                self._phase = None
            return

        # Any other state is an in-flight phase (starting / taking_off /
        # running / returning / landing, or anything a future phase adds).
        if not self._running:
            self._begin(now, state)
            return
        if state != self._phase:
            self._close_phase(now)
            self._phase = state
            self._phase_started = now

    def reset(self) -> None:
        """Drop all timing state, as if no mission had ever run."""
        self._start = None
        self._frozen_elapsed = 0.0
        self._running = False
        self._phase = None
        self._phase_started = 0.0
        self._phases = []

    # -- internals -----------------------------------------------------

    def _begin(self, now: float, state: str) -> None:
        self._start = now
        self._frozen_elapsed = 0.0
        self._running = True
        self._phases = []
        self._phase = state
        self._phase_started = now

    def _close_phase(self, now: float) -> None:
        if self._phase is not None:
            self._phases.append((self._phase, now - self._phase_started))


def format_duration(seconds: float) -> str:
    """Human-readable mission duration: '4m 07s', '1h 02m 30s', '12s'.

    Used for the log breakdown; the GCS formats its own display from the
    raw float so it can render seconds ticking without reparsing a string.
    """
    seconds = max(0.0, float(seconds))
    hours, rem = divmod(int(seconds), 3600)
    minutes, secs = divmod(rem, 60)
    if hours:
        return f'{hours}h {minutes:02d}m {secs:02d}s'
    if minutes:
        return f'{minutes}m {secs:02d}s'
    return f'{secs}s'
