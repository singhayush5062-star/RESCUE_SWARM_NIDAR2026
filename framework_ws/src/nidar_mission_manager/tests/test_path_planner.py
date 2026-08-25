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


def test_waypoint_count_matches_hand_computed_lines():
    # width=150 >= height=100 -> default heading runs lines along the 150m
    # (longer) axis, spaced across the 100m (shorter) axis.
    zone = make_rect_latlon(150.0, 100.0)
    wps = path_planner.generate_lawnmower_path(zone, *ORIGIN,
                                                scan_altitude_m=25.0, camera_hfov_deg=60.0, overlap_pct=20.0)
    # swath=28.87m, spacing=23.10m, n_lines=ceil(100/23.10)=5 -> 10 waypoints
    assert len(wps) == 10


def test_boustrophedon_alternates_direction():
    zone = make_rect_latlon(28.0, 100.0)
    wps = path_planner.generate_lawnmower_path(zone, *ORIGIN)
    zone_enu = geo_utils.latlon_to_enu([(lat, lon) for lat, lon, _ in wps], *ORIGIN)
    directions = [math.copysign(1, zone_enu[i + 1][0] - zone_enu[i][0]) for i in range(0, len(zone_enu) - 1, 2)]
    assert all(directions[i] != directions[i + 1] for i in range(len(directions) - 1))


def test_tiny_zone_still_returns_one_line():
    zone = make_rect_latlon(5.0, 5.0)      # far smaller than any realistic swath
    wps = path_planner.generate_lawnmower_path(zone, *ORIGIN)
    assert len(wps) >= 2


def test_invalid_overlap_raises():
    zone = make_rect_latlon(50, 50)
    with pytest.raises(ValueError):
        path_planner.generate_lawnmower_path(zone, *ORIGIN, overlap_pct=100)
