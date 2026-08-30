"""Merge rules for survivor identity."""
import pytest

from nidar_mission_manager.survivor_aggregator import (
    SurvivorRegistry, haversine_m)

LAT, LON = 28.682412, 77.499734
M = 1.0 / 111320.0   # roughly one metre of latitude, in degrees


def test_haversine_metre_scale():
    assert haversine_m(LAT, LON, LAT + M, LON) == pytest.approx(1.0, rel=1e-3)


def test_first_sighting_creates_a_record():
    r = SurvivorRegistry(dedup_radius_m=5.0)
    rec, is_new = r.observe(LAT, LON, 0.0, 0.9, 'drone0')
    assert is_new and rec.survivor_id == 0 and len(r) == 1


def test_nearby_sighting_merges():
    r = SurvivorRegistry(dedup_radius_m=5.0)
    r.observe(LAT, LON, 0.0, 0.9, 'drone0')
    rec, is_new = r.observe(LAT + 2 * M, LON, 0.0, 0.8, 'drone1')
    assert not is_new and len(r) == 1 and rec.observations == 2


def test_distant_sighting_is_a_separate_survivor():
    r = SurvivorRegistry(dedup_radius_m=5.0)
    r.observe(LAT, LON, 0.0, 0.9, 'drone0')
    _, is_new = r.observe(LAT + 20 * M, LON, 0.0, 0.9, 'drone0')
    assert is_new and len(r) == 2


def test_two_drones_seeing_one_person_are_both_credited():
    r = SurvivorRegistry(dedup_radius_m=5.0)
    r.observe(LAT, LON, 0.0, 0.9, 'drone0')
    rec, _ = r.observe(LAT + M, LON, 0.0, 0.8, 'drone2')
    assert rec.detecting_drones == ['drone0', 'drone2']


def test_a_drone_is_credited_once_however_many_sightings():
    r = SurvivorRegistry(dedup_radius_m=5.0)
    for _ in range(5):
        rec, _ = r.observe(LAT, LON, 0.0, 0.9, 'drone0')
    assert rec.detecting_drones == ['drone0'] and rec.observations == 5


def test_key_merges_even_beyond_the_radius():
    """A tracked person who walks past the dedup radius must stay one
    survivor: the tracker followed them, which outranks distance."""
    r = SurvivorRegistry(dedup_radius_m=2.0)
    r.observe(LAT, LON, 0.0, 0.9, 'drone0', key='drone0:7')
    _, is_new = r.observe(LAT + 30 * M, LON, 0.0, 0.9, 'drone0', key='drone0:7')
    assert not is_new and len(r) == 1


def test_different_keys_within_the_radius_still_merge_geographically():
    """A lost-and-reacquired track gets a new id; geography must rejoin it."""
    r = SurvivorRegistry(dedup_radius_m=5.0)
    r.observe(LAT, LON, 0.0, 0.9, 'drone0', key='drone0:7')
    _, is_new = r.observe(LAT + M, LON, 0.0, 0.9, 'drone0', key='drone0:9')
    assert not is_new and len(r) == 1


def test_confidence_is_the_best_look_not_the_average():
    r = SurvivorRegistry(dedup_radius_m=5.0)
    r.observe(LAT, LON, 0.0, 0.95, 'drone0')
    rec, _ = r.observe(LAT, LON, 0.0, 0.20, 'drone0')
    assert rec.confidence == pytest.approx(0.95)


def test_position_is_confidence_weighted_so_a_weak_sighting_barely_moves_it():
    r = SurvivorRegistry(dedup_radius_m=10.0)
    r.observe(LAT, LON, 0.0, 0.99, 'drone0')
    rec, _ = r.observe(LAT + 4 * M, LON, 0.0, 0.01, 'drone0')
    assert haversine_m(LAT, LON, rec.latitude, rec.longitude) < 0.1


def test_averaging_converges_on_the_truth_despite_jitter():
    """Ten sightings scattered +/- 2 m around a true position must average to
    well inside the 3 m Phase 3.4 accuracy target."""
    r = SurvivorRegistry(dedup_radius_m=10.0)
    for i, offset in enumerate([-2, 1.5, -1, 2, -1.5, 0.5, 1, -0.5, 1.8, -1.8]):
        r.observe(LAT + offset * M, LON + offset * M, 0.0, 0.8, 'drone0')
    rec = r.records[0]
    assert haversine_m(LAT, LON, rec.latitude, rec.longitude) < 0.5


def test_all_zero_confidence_does_not_produce_nan():
    r = SurvivorRegistry(dedup_radius_m=5.0)
    r.observe(LAT, LON, 0.0, 0.0, 'drone0')
    rec, _ = r.observe(LAT, LON, 0.0, 0.0, 'drone0')
    assert rec.latitude == pytest.approx(LAT)


def test_ids_are_stable_and_monotonic():
    r = SurvivorRegistry(dedup_radius_m=1.0)
    a, _ = r.observe(LAT, LON, 0.0, 0.9, 'drone0')
    b, _ = r.observe(LAT + 50 * M, LON, 0.0, 0.9, 'drone0')
    again, is_new = r.observe(LAT, LON, 0.0, 0.9, 'drone0')
    assert (a.survivor_id, b.survivor_id) == (0, 1)
    assert not is_new and again.survivor_id == 0
    assert r.find(1) is b


# --- regression: the radius must not swallow distinct survivors -----------
#
# Measured on a 14-survivor coverage flight, the original 5.0 m default
# merged away five genuinely distinct people. These pin the constraint from
# both sides so a future radius change has to stay inside the window.

from nidar_mission_manager.survivor_aggregator import (  # noqa: E402
    DEFAULT_GLOBAL_DEDUP_RADIUS_M, DEFAULT_LOCAL_DEDUP_RADIUS_M)

#: Closest pair of distinct survivors in the arena, and the minimum
#: separation the GCS enforces when it scatters survivors randomly
#: (launchSiteManager.generateRandomPointsInPolygon).
MIN_SURVIVOR_SEPARATION_M = 3.0

#: Worst per-view geotag error measured on that same flight, excluding
#: records the over-merging had already corrupted.
MEASURED_GEOTAG_ERROR_M = 1.15


def test_dedup_radii_stay_below_the_minimum_survivor_separation():
    """Above this and two people standing 3 m apart become one survivor."""
    assert DEFAULT_GLOBAL_DEDUP_RADIUS_M < MIN_SURVIVOR_SEPARATION_M
    assert DEFAULT_LOCAL_DEDUP_RADIUS_M < MIN_SURVIVOR_SEPARATION_M


def test_dedup_radii_stay_above_the_measured_geotag_error():
    """Below this and one person seen twice becomes two survivors."""
    assert DEFAULT_GLOBAL_DEDUP_RADIUS_M > MEASURED_GEOTAG_ERROR_M
    assert DEFAULT_LOCAL_DEDUP_RADIUS_M > MEASURED_GEOTAG_ERROR_M


def test_two_survivors_three_metres_apart_stay_distinct():
    """The exact case that lost survivor_1 behind survivor_2."""
    r = SurvivorRegistry(dedup_radius_m=DEFAULT_GLOBAL_DEDUP_RADIUS_M)
    r.observe(LAT, LON, 0.0, 0.95, 'drone0')
    _, is_new = r.observe(LAT + MIN_SURVIVOR_SEPARATION_M * M, LON, 0.0, 0.95, 'drone1')
    assert is_new and len(r) == 2


def test_one_survivor_seen_by_two_drones_at_measured_error_still_merges():
    """The other side of the window: two drones' independent estimates of the
    same person, separated by the worst error actually measured."""
    r = SurvivorRegistry(dedup_radius_m=DEFAULT_GLOBAL_DEDUP_RADIUS_M)
    r.observe(LAT, LON, 0.0, 0.95, 'drone0')
    rec, is_new = r.observe(LAT + MEASURED_GEOTAG_ERROR_M * M, LON, 0.0, 0.90, 'drone1')
    assert not is_new and len(r) == 1
    assert rec.detecting_drones == ['drone0', 'drone1']
