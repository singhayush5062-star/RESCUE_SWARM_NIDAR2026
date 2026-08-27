"""Unit tests for the mission clock's timing rules.

Uses a fake clock rather than real sleeps: these assert *rules* (when the
clock starts, what freezes it, what a new run resets), and tying them to
wall time would make them both slow and flaky.
"""

import pytest

from nidar_mission_clock.clock import MissionClock, format_duration


class FakeClock:
    """A monotonic clock the test drives by hand."""

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


@pytest.fixture
def fake():
    return FakeClock()


@pytest.fixture
def clock(fake):
    return MissionClock(now_fn=fake)


def test_idle_and_loaded_do_not_start_the_clock(clock, fake):
    clock.on_state('idle')
    fake.advance(30)
    clock.on_state('loaded')
    fake.advance(30)
    assert not clock.running
    assert clock.elapsed() == 0.0


def test_clock_starts_on_first_in_flight_state(clock, fake):
    clock.on_state('loaded')
    clock.on_state('starting')
    assert clock.running
    fake.advance(12.5)
    assert clock.elapsed() == pytest.approx(12.5)


def test_elapsed_freezes_at_the_final_total_on_complete(clock, fake):
    clock.on_state('starting')
    fake.advance(60)
    clock.on_state('complete')
    assert not clock.running
    assert clock.elapsed() == pytest.approx(60)
    # Time keeps passing; the reported total must not.
    fake.advance(300)
    assert clock.elapsed() == pytest.approx(60)


def test_error_also_freezes_the_clock(clock, fake):
    clock.on_state('starting')
    fake.advance(9)
    clock.on_state('error')
    assert not clock.running
    assert clock.elapsed() == pytest.approx(9)


def test_loading_a_new_mission_keeps_the_last_total_visible(clock, fake):
    """The operator sets up the next run while still reading the last run's
    time -- 'loaded' must not wipe it."""
    clock.on_state('starting')
    fake.advance(42)
    clock.on_state('complete')
    clock.on_state('loaded')
    assert clock.elapsed() == pytest.approx(42)


def test_a_new_run_resets_the_clock(clock, fake):
    clock.on_state('starting')
    fake.advance(42)
    clock.on_state('complete')
    clock.on_state('loaded')
    clock.on_state('starting')
    assert clock.running
    assert clock.elapsed() == pytest.approx(0.0)
    fake.advance(5)
    assert clock.elapsed() == pytest.approx(5)


def test_phase_durations_sum_to_elapsed(clock, fake):
    clock.on_state('starting')
    fake.advance(10)
    clock.on_state('taking_off')
    fake.advance(20)
    clock.on_state('running')
    fake.advance(120)
    clock.on_state('returning')
    fake.advance(30)
    clock.on_state('landing')
    fake.advance(15)
    clock.on_state('complete')

    phases = clock.phase_durations()
    assert [name for name, _ in phases] == [
        'starting', 'taking_off', 'running', 'returning', 'landing']
    assert [pytest.approx(d) for _, d in phases] == [10, 20, 120, 30, 15]
    assert sum(d for _, d in phases) == pytest.approx(clock.elapsed())


def test_mid_run_breakdown_includes_the_phase_still_in_progress(clock, fake):
    clock.on_state('starting')
    fake.advance(10)
    clock.on_state('running')
    fake.advance(7)
    phases = clock.phase_durations()
    assert [name for name, _ in phases] == ['starting', 'running']
    assert sum(d for _, d in phases) == pytest.approx(clock.elapsed())


def test_repeated_state_is_not_a_new_phase(clock, fake):
    """The JSON-waypoint flow republishes 'running' once per waypoint --
    those are progress updates inside one phase, not phase changes."""
    clock.on_state('starting')
    fake.advance(5)
    for _ in range(4):
        clock.on_state('running')
        fake.advance(10)
    clock.on_state('complete')
    phases = clock.phase_durations()
    assert [name for name, _ in phases] == ['starting', 'running']
    assert dict(phases)['running'] == pytest.approx(40)


def test_unknown_state_is_treated_as_an_in_flight_phase(clock, fake):
    """Adding a state to the executor must never break the clock."""
    clock.on_state('some_future_phase')
    assert clock.running
    fake.advance(3)
    assert clock.elapsed() == pytest.approx(3)


def test_terminal_state_without_a_run_does_nothing(clock):
    clock.on_state('complete')
    assert not clock.running
    assert clock.elapsed() == 0.0


def test_reset_clears_everything(clock, fake):
    clock.on_state('starting')
    fake.advance(30)
    clock.on_state('complete')
    clock.reset()
    assert clock.elapsed() == 0.0
    assert clock.phase_durations() == []


@pytest.mark.parametrize('seconds,expected', [
    (0, '0s'),
    (9.4, '9s'),
    (59, '59s'),
    (60, '1m 00s'),
    (247, '4m 07s'),
    (3600, '1h 00m 00s'),
    (3750, '1h 02m 30s'),
    (-5, '0s'),
])
def test_format_duration(seconds, expected):
    assert format_duration(seconds) == expected
