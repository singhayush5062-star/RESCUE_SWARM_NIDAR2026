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


def zone_centroid_enu(zone):
    enu = geo_utils.latlon_to_enu(zone, *ORIGIN)
    return (sum(p[0] for p in enu) / len(enu), sum(p[1] for p in enu) / len(enu))


def test_grid_split_makes_square_zones_not_strips():
    """A square arena split 4 ways must tile 2x2, not into 4 thin ribbons.

    Strips (the old behaviour) gave 100x25 zones; flown along their long
    axis those need only ~2 lawnmower passes at realistic scan altitudes,
    producing a degenerate two-line path instead of a real serpentine.
    """
    boundary = make_square_latlon(100.0)
    zones = zone_splitter.split_boundary(boundary, num_zones=4,
                                          origin_lat=ORIGIN[0], origin_lon=ORIGIN[1], origin_alt=ORIGIN[2])
    assert len(zones) == 4
    for z in zones:
        enu = geo_utils.latlon_to_enu(z, *ORIGIN)
        w = max(p[0] for p in enu) - min(p[0] for p in enu)
        h = max(p[1] for p in enu) - min(p[1] for p in enu)
        assert math.isclose(w, 50.0, rel_tol=0.02), f'expected 50m-wide cell, got {w:.1f}'
        assert math.isclose(h, 50.0, rel_tol=0.02), f'expected 50m-tall cell, got {h:.1f}'
    # All four centroids distinct -> a real 2x2 tiling, not 4 overlapping cells.
    assert len({tuple(round(c, 3) for c in zone_centroid_enu(z)) for z in zones}) == 4


def test_strip_split_still_available():
    """grid=False must still produce N parallel strips.

    Asserts the strip *shape* rather than a specific axis: for a square
    boundary neither axis is longer, so which one gets sliced is an
    arbitrary tie-break, and pinning it makes the test fail on a change that
    breaks nothing.
    """
    boundary = make_square_latlon(100.0)
    zones = zone_splitter.split_boundary(boundary, num_zones=4, origin_lat=ORIGIN[0],
                                          origin_lon=ORIGIN[1], origin_alt=ORIGIN[2], grid=False)
    assert len(zones) == 4
    for zone in zones:
        enu = geo_utils.latlon_to_enu(zone, *ORIGIN)
        w = max(p[0] for p in enu) - min(p[0] for p in enu)
        h = max(p[1] for p in enu) - min(p[1] for p in enu)
        short, long_ = sorted((w, h))
        assert math.isclose(short, 25.0, rel_tol=0.02), f'expected a 25m-thick strip, got {short:.1f}'
        assert math.isclose(long_, 100.0, rel_tol=0.02), f'strip should span the boundary, got {long_:.1f}'

    # Four distinct strips, not four copies of the same one.
    assert len({tuple(round(c, 3) for c in zone_centroid_enu(z)) for z in zones}) == 4


def test_assign_nearest_zones_matches_by_proximity():
    """Each drone must get the zone nearest its own launch position, not
    whichever zone happened to sit at its index in the zone list."""
    boundary = make_square_latlon(100.0)
    zones = zone_splitter.split_boundary(boundary, num_zones=4,
                                          origin_lat=ORIGIN[0], origin_lon=ORIGIN[1], origin_alt=ORIGIN[2])
    centroids = [zone_centroid_enu(z) for z in zones]

    # Drones parked exactly on the zone centroids but handed to the assigner
    # in REVERSE zone order, so a naive positional zip would pair every one
    # of them with the wrong zone.
    order = list(reversed(range(4)))
    drone_positions_enu = {f'drone{k}': centroids[i] for k, i in enumerate(order)}
    drone_positions = {ns: geo_utils.enu_to_latlon([xy], *ORIGIN)[0]
                        for ns, xy in drone_positions_enu.items()}

    assignment = zone_splitter.assign_nearest_zones(drone_positions, zones)
    assert set(assignment.keys()) == set(drone_positions.keys())
    # Every drone ends up on the zone whose centroid it was standing on.
    for ns, zone in assignment.items():
        got = zone_centroid_enu(zone)
        want = drone_positions_enu[ns]
        assert math.isclose(got[0], want[0], abs_tol=0.5) and math.isclose(got[1], want[1], abs_tol=0.5)
    # And no zone handed out twice.
    assert len({tuple(round(c, 3) for c in zone_centroid_enu(z)) for z in assignment.values()}) == 4
