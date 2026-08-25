import math

import pytest

from nidar_mission_manager import geo_utils, zone_splitter

ORIGIN = (28.682412, 77.499734, 100.0)   # world_swarm.yaml's actual origin


def polygon_area(vertices):
    """Shoelace formula, test-only helper (no shapely)."""
    n = len(vertices)
    s = sum(vertices[i][0] * vertices[(i + 1) % n][1] - vertices[(i + 1) % n][0] * vertices[i][1]
            for i in range(n))
    return abs(s) / 2.0


def make_square_latlon(size_m):
    corners_enu = [(0, 0), (size_m, 0), (size_m, size_m), (0, size_m)]
    return geo_utils.enu_to_latlon(corners_enu, *ORIGIN)


def test_split_rectangle_into_4_equal_zones():
    boundary = make_square_latlon(100.0)
    zones = zone_splitter.split_boundary(boundary, num_zones=4,
                                          origin_lat=ORIGIN[0], origin_lon=ORIGIN[1], origin_alt=ORIGIN[2])
    assert len(zones) == 4
    zones_enu = [geo_utils.latlon_to_enu(z, *ORIGIN) for z in zones]
    areas = [polygon_area(z) for z in zones_enu]
    for a in areas:
        assert math.isclose(a, 2500.0, rel_tol=0.01)          # 100x100/4
    assert math.isclose(sum(areas), 10000.0, rel_tol=0.01)     # full coverage, no gaps/overlap


def test_split_single_zone_returns_original():
    boundary = make_square_latlon(50.0)
    zones = zone_splitter.split_boundary(boundary, num_zones=1,
                                          origin_lat=ORIGIN[0], origin_lon=ORIGIN[1], origin_alt=ORIGIN[2])
    assert len(zones) == 1


def test_split_invalid_num_zones_raises():
    boundary = make_square_latlon(50.0)
    with pytest.raises(ValueError):
        zone_splitter.split_boundary(boundary, num_zones=0, origin_lat=ORIGIN[0], origin_lon=ORIGIN[1])
