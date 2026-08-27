import math

import pytest

from nidar_mission_manager import geo_utils, path_planner

ORIGIN = (28.682412, 77.499734, 100.0)


def make_rect_latlon(w, h):
    corners_enu = [(0, 0), (w, 0), (w, h), (0, h)]
    return geo_utils.enu_to_latlon(corners_enu, *ORIGIN)


def test_waypoints_stay_inside_zone_bbox():
    zone = make_rect_latlon(80, 40)
    wps = path_planner.generate_lawnmower_path(zone, *ORIGIN)
    zone_enu = geo_utils.latlon_to_enu(zone, *ORIGIN)
    xs = [p[0] for p in zone_enu]; ys = [p[1] for p in zone_enu]
    for lat, lon, _alt in wps:
        (x, y), = geo_utils.latlon_to_enu([(lat, lon)], *ORIGIN)
        assert min(xs) - 0.5 <= x <= max(xs) + 0.5
        assert min(ys) - 0.5 <= y <= max(ys) + 0.5


def test_camera_footprint_matches_hd_camera_spec():
    """60 deg HFOV at 1280x960 must give the documented 11.55m x 8.66m ground
    frame at 10m -- the number the coverage spacing is derived from."""
    long_m, short_m = path_planner.ground_footprint_m(10.0)
    assert math.isclose(long_m, 11.55, abs_tol=0.01)
    assert math.isclose(short_m, 8.66, abs_tol=0.01)
    assert math.isclose(path_planner.camera_vfov_deg(), 46.83, abs_tol=0.01)


def test_swath_is_conservative_short_edge():
    """Spacing must come from the SHORT footprint edge. AS2 flies legs with
    yaw held fixed (KEEP_YAW), so the wide edge is not guaranteed to lie
    across-track; planning on it leaves gaps whenever the camera is rotated."""
    long_m, short_m = path_planner.ground_footprint_m(10.0)
    assert math.isclose(path_planner.swath_width_m(10.0), short_m)
    assert math.isclose(path_planner.swath_width_m(10.0, conservative=False), long_m)


def test_waypoint_count_matches_hand_computed_lines():
    # width=150 >= height=100 -> default heading runs lines along the 150m
    # (longer) axis, spaced across the 100m (shorter) axis.
    zone = make_rect_latlon(150.0, 100.0)
    wps = path_planner.generate_lawnmower_path(zone, *ORIGIN,
                                                scan_altitude_m=25.0, camera_hfov_deg=60.0, overlap_pct=20.0)
    # Worked example, conservative (short-edge) swath at 25m:
    #   VFOV = 46.83 deg -> swath = 2*25*tan(46.83/2) = 21.65m
    #   spacing = 21.65 * (1 - 0.20) = 17.32m
    #   n_lines = ceil(100 / 17.32) = 6  -> 12 waypoints
    assert len(wps) == 12


def test_boustrophedon_alternates_direction():
    """Consecutive scan lines must run in opposite directions.

    Measured along whichever axis the lines actually run, not a hardcoded
    one: flight lines follow the zone's *longer* axis (fewer turns), so a
    28m x 100m zone is scanned along Y, and testing the X component would
    only ever see the constant across-track offset -- which is 0 for every
    leg and so trivially "matches", hiding a genuinely broken serpentine.
    """
    zone = make_rect_latlon(28.0, 100.0)
    wps = path_planner.generate_lawnmower_path(zone, *ORIGIN)
    zone_enu = geo_utils.latlon_to_enu([(lat, lon) for lat, lon, _ in wps], *ORIGIN)

    legs = [(zone_enu[i], zone_enu[i + 1]) for i in range(0, len(zone_enu) - 1, 2)]
    assert len(legs) >= 2, 'need at least two legs to alternate'

    # Along-track axis = the one the first leg actually travels along.
    a, b = legs[0]
    axis = 0 if abs(b[0] - a[0]) >= abs(b[1] - a[1]) else 1
    assert abs(b[axis] - a[axis]) > 1.0, 'legs must have real length'

    directions = [math.copysign(1, end[axis] - start[axis]) for start, end in legs]
    assert all(directions[i] != directions[i + 1] for i in range(len(directions) - 1))


def test_tiny_zone_still_returns_one_line():
    zone = make_rect_latlon(5.0, 5.0)      # far smaller than any realistic swath
    wps = path_planner.generate_lawnmower_path(zone, *ORIGIN)
    assert len(wps) >= 2


def test_invalid_overlap_raises():
    zone = make_rect_latlon(50, 50)
    with pytest.raises(ValueError):
        path_planner.generate_lawnmower_path(zone, *ORIGIN, overlap_pct=100)
