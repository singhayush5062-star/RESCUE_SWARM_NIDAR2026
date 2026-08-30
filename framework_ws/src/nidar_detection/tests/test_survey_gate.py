"""The survey gate: when may the detector look?

Each case is a state the swarm genuinely passes through in a mission, and
the reason it must or must not be inferred on.
"""
import pytest

from nidar_detection.survey_gate import (
    DEFAULT_ALTITUDE_TOLERANCE_M, SURVEY_STATE, is_surveying)

SCAN_ALT = 14.0


def test_surveying_at_scan_altitude():
    assert is_surveying(SURVEY_STATE, SCAN_ALT, SCAN_ALT)


def test_small_altitude_hold_error_is_tolerated():
    """The controller does not hold altitude exactly; a gate that demanded it
    would flicker closed through every survey leg."""
    assert is_surveying(SURVEY_STATE, SCAN_ALT + 0.9, SCAN_ALT)
    assert is_surveying(SURVEY_STATE, SCAN_ALT - 0.9, SCAN_ALT)


@pytest.mark.parametrize('state', ['idle', 'loaded', 'starting', 'taking_off',
                                   'returning', 'landing', 'complete', 'error'])
def test_every_non_survey_phase_is_closed(state):
    """Takeoff, return and landing all happen over the launch box, where the
    cameras see other drones. Those phases produced the phantom survivors."""
    assert not is_surveying(state, SCAN_ALT, SCAN_ALT)


def test_climb_below_scan_altitude_is_closed():
    assert not is_surveying(SURVEY_STATE, 6.2, SCAN_ALT)


def test_staggered_return_altitudes_are_closed():
    """Return-to-launch flies home at scan_alt + 2.5 m per drone index, over
    the launch box. Those must not be inferred on even though the mission
    state may still read as flying."""
    for idx in (1, 2, 3):
        alt = SCAN_ALT + idx * 2.5
        assert not is_surveying('returning', alt, SCAN_ALT)


def test_drone_zero_returning_at_exactly_scan_altitude_is_closed():
    """The case an altitude-only gate would miss: drone0's return leg is at
    scan altitude exactly, straight over the launch box. Only the phase check
    catches it."""
    assert not is_surveying('returning', SCAN_ALT, SCAN_ALT)


def test_unknown_mission_state_is_closed():
    """Before any status arrives, nothing is known. Defaulting open here is
    exactly how the old always-on behaviour would creep back."""
    assert not is_surveying(None, SCAN_ALT, SCAN_ALT)


def test_unknown_altitude_is_closed():
    assert not is_surveying(SURVEY_STATE, None, SCAN_ALT)


def test_unknown_scan_altitude_is_closed():
    assert not is_surveying(SURVEY_STATE, SCAN_ALT, None)


def test_tolerance_boundary_is_inclusive():
    assert is_surveying(SURVEY_STATE, SCAN_ALT + DEFAULT_ALTITUDE_TOLERANCE_M, SCAN_ALT)
    assert not is_surveying(SURVEY_STATE,
                            SCAN_ALT + DEFAULT_ALTITUDE_TOLERANCE_M + 0.01, SCAN_ALT)


def test_tolerance_is_configurable():
    assert not is_surveying(SURVEY_STATE, SCAN_ALT + 3.0, SCAN_ALT)
    assert is_surveying(SURVEY_STATE, SCAN_ALT + 3.0, SCAN_ALT, tolerance_m=4.0)


# --- zone and mapping-area geometry --------------------------------------

from nidar_detection.survey_gate import is_inside_area, point_in_polygon  # noqa: E402

#: A ~110 m x ~97 m box near the sim origin, in (lat, lon).
BOX = [(28.68200, 77.49900), (28.68200, 77.50000),
       (28.68300, 77.50000), (28.68300, 77.49900)]


def test_point_inside_zone():
    assert point_in_polygon(28.68250, 77.49950, BOX)


def test_point_outside_zone():
    assert not point_in_polygon(28.68400, 77.49950, BOX)


def test_no_zone_is_not_everywhere():
    """A drone that has not received its allocation must not be treated as
    being inside it -- that would reopen the gate during transit."""
    assert not point_in_polygon(28.68250, 77.49950, [])
    assert not point_in_polygon(28.68250, 77.49950, [(0, 0), (1, 1)])


def test_area_accepts_inside():
    assert is_inside_area(28.68250, 77.49950, BOX, 2.0)


def test_area_rejects_well_outside():
    """The measured case: a record several metres beyond the mapping area.
    Offset from the box's northern EDGE (28.68300), not from its middle."""
    outside = 28.68300 + 4.5 / 111320.0
    assert not is_inside_area(outside, 77.49950, BOX, 2.0)


def test_area_margin_keeps_a_survivor_just_outside_the_line():
    """Geotag error is 0.40 m mean / 1.16 m worst case, so a survivor standing
    on the boundary can project just outside it. Rejecting them would turn a
    position error into a missing person."""
    just_out = 28.68300 + 1.0 / 111320.0     # 1 m north of the top edge
    assert is_inside_area(just_out, 77.49950, BOX, 2.0)
    assert not is_inside_area(just_out, 77.49950, BOX, 0.5)


def test_unknown_boundary_does_not_filter():
    """Inverted from the zone rule, deliberately: no boundary must mean 'do
    not filter', never 'reject everything'."""
    assert is_inside_area(28.68250, 77.49950, [], 2.0)
