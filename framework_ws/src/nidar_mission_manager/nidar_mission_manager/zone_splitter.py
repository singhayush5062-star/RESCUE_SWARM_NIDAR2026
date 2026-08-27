"""Splits a boundary polygon into N sub-polygons (one per drone) by
bounding-box strip slicing + Sutherland-Hodgman clipping.

Sutherland-Hodgman is exact here because every clip window this module
builds is an axis-aligned rectangle (convex) -- it correctly handles a
concave *subject* polygon (the boundary) even though it would not, in
general, handle a concave *clip* polygon.
"""
import math
from typing import List, Tuple

from . import geo_utils

Point = Tuple[float, float]

_EPS = 1e-6           # generic float-comparison slack
_PAD_M = 5.0           # perpendicular padding on the clip rectangle, in meters,
                        # so a polygon vertex that lands exactly on the bbox's
                        # own edge (common for axis-aligned rectangular arenas)
                        # is never dropped by a strict inside/outside test


def split_boundary(boundary_latlon: List[Point], num_zones: int,
                    origin_lat: float, origin_lon: float, origin_alt: float = 0.0,
                    grid: bool = True) -> List[List[Point]]:
    """Entry point. boundary_latlon: [(lat, lon), ...], polygon, >= 3 points.
    Returns num_zones sub-polygons, each [(lat, lon), ...], covering the
    original boundary with no gaps and (for convex/rectangular input) no
    overlap. Raises ValueError on bad input or a degenerate split result.

    grid=True (default) tiles the boundary into the rows x cols arrangement
    that makes each zone as close to square as possible. grid=False keeps
    the original single-axis strip split.

    Why grid is the default: strip splitting turns a roughly-square arena
    into long thin ribbons (a 33.9m square split 4 ways gives 33.9m x 8.5m
    zones). Flown along their long axis, such a ribbon needs only ~2 passes
    at typical scan altitudes -- technically full coverage, but a degenerate
    two-line "lawnmower" rather than a real serpentine, and it drags every
    drone across the arena's full width. Square-ish zones give each drone a
    compact patch near its own launch position, a recognisable multi-line
    boustrophedon, and shorter transits.
    """
    if num_zones < 1:
        raise ValueError('num_zones must be >= 1')
    if len(boundary_latlon) < 3:
        raise ValueError('boundary must have at least 3 vertices')

    boundary_enu = geo_utils.latlon_to_enu(boundary_latlon, origin_lat, origin_lon, origin_alt)
    zones_enu = (_split_polygon_enu_grid(boundary_enu, num_zones) if grid
                 else _split_polygon_enu(boundary_enu, num_zones))
    return [geo_utils.enu_to_latlon(z, origin_lat, origin_lon, origin_alt) for z in zones_enu]


def _best_grid_shape(width: float, height: float, num_zones: int) -> Tuple[int, int]:
    """Pick (rows, cols) with rows*cols == num_zones whose resulting cells are
    closest to square for a width x height bounding box."""
    best = None
    for cols in range(1, num_zones + 1):
        if num_zones % cols:
            continue
        rows = num_zones // cols
        cell_w, cell_h = width / cols, height / rows
        if cell_w <= 0 or cell_h <= 0:
            continue
        # Deviation from square, scale-free so a 2:1 cell scores the same as 1:2.
        score = abs(math.log(cell_w / cell_h))
        if best is None or score < best[0]:
            best = (score, rows, cols)
    if best is None:
        return (1, num_zones)
    return best[1], best[2]


def _split_polygon_enu_grid(polygon: List[Point], num_zones: int) -> List[List[Point]]:
    if num_zones == 1:
        return [polygon]

    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    width, height = max_x - min_x, max_y - min_y

    rows, cols = _best_grid_shape(width, height, num_zones)
    cell_w, cell_h = width / cols, height / rows

    zones = []
    index = 0
    for r in range(rows):
        for c in range(cols):
            # Pad only where a cell touches the bounding box, so a vertex
            # lying exactly on the arena edge is never dropped by the strict
            # inside/outside test -- without padding interior seams, which
            # would make neighbouring zones overlap.
            x0 = (min_x - _PAD_M) if c == 0 else min_x + c * cell_w
            x1 = (max_x + _PAD_M) if c == cols - 1 else min_x + (c + 1) * cell_w
            y0 = (min_y - _PAD_M) if r == 0 else min_y + r * cell_h
            y1 = (max_y + _PAD_M) if r == rows - 1 else min_y + (r + 1) * cell_h
            clip = _rect(x0, y0, x1, y1)
            zones.append(_require_nonempty(_sutherland_hodgman(polygon, clip), index))
            index += 1
    return zones


def _split_polygon_enu(polygon: List[Point], num_zones: int) -> List[List[Point]]:
    if num_zones == 1:
        return [polygon]

    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    width, height = max_x - min_x, max_y - min_y

    zones = []
    if width >= height:
        step = width / num_zones
        for i in range(num_zones):
            x0 = min_x + i * step
            x1 = max_x if i == num_zones - 1 else min_x + (i + 1) * step
            clip = _rect(x0, min_y - _PAD_M, x1, max_y + _PAD_M)
            zones.append(_require_nonempty(_sutherland_hodgman(polygon, clip), i))
    else:
        step = height / num_zones
        for i in range(num_zones):
            y0 = min_y + i * step
            y1 = max_y if i == num_zones - 1 else min_y + (i + 1) * step
            clip = _rect(min_x - _PAD_M, y0, max_x + _PAD_M, y1)
            zones.append(_require_nonempty(_sutherland_hodgman(polygon, clip), i))
    return zones


def _rect(x0, y0, x1, y1) -> List[Point]:
    # CCW winding -- required by _sutherland_hodgman's `inside()` test below.
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def _require_nonempty(zone: List[Point], index: int) -> List[Point]:
    if len(zone) < 3:
        raise ValueError(f'zone splitting produced a degenerate/empty zone at index {index}')
    return zone


def _sutherland_hodgman(subject: List[Point], clip: List[Point]) -> List[Point]:
    """Standard Sutherland-Hodgman polygon clip. `clip` MUST be convex and
    wound counter-clockwise. `subject` may be any simple polygon (convex or
    concave). Returns [] if there is no overlap."""

    def inside(p, a, b):
        return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= -_EPS

    def intersect(p1, p2, a, b):
        x1, y1 = p1; x2, y2 = p2; x3, y3 = a; x4, y4 = b
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-12:
            return p2
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))

    output = list(subject)
    cp1 = clip[-1]
    for cp2 in clip:
        input_list, output = output, []
        if not input_list:
            break
        s = input_list[-1]
        for e in input_list:
            e_in, s_in = inside(e, cp1, cp2), inside(s, cp1, cp2)
            if e_in:
                if not s_in:
                    output.append(intersect(s, e, cp1, cp2))
                output.append(e)
            elif s_in:
                output.append(intersect(s, e, cp1, cp2))
            s = e
        cp1 = cp2
    return output


def assign_nearest_zones(drone_positions: dict, zones_latlon: List[List[Point]]) -> dict:
    """Greedy nearest-zone assignment: each drone (in the order given) claims
    whichever remaining zone's centroid is closest to its own launch position.

    Without this, a plain positional pairing (namespace[i] <-> zones[i]) sends
    drones to zones based on the arbitrary order they appear in the world
    config, independent of where they actually are -- a drone launched on one
    side of the arena could get the zone furthest from it while a drone right
    next to that zone gets sent elsewhere. Not a globally-optimal assignment
    (that's a bipartite matching problem); greedy nearest-first is enough to
    fix "drones fly to the wrong side of the arena" at the scale this project
    operates at (a handful of drones), without pulling in an optimization
    library for a difference that wouldn't be visible at this scale anyway.

    :param drone_positions: {namespace: (lat, lon)}, iteration order is the
        greedy claim order (first drone gets first pick).
    :param zones_latlon: zones as returned by split_boundary(), matched 1:1
        in count with drone_positions.
    :return: {namespace: zone} using the same zone objects passed in.
    """
    def centroid(zone: List[Point]) -> Point:
        lats = [p[0] for p in zone]
        lons = [p[1] for p in zone]
        return (sum(lats) / len(lats), sum(lons) / len(lons))

    def dist_sq(a: Point, b: Point) -> float:
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2

    remaining = list(zones_latlon)
    assignment = {}
    for ns, pos in drone_positions.items():
        best_i = min(range(len(remaining)), key=lambda i: dist_sq(pos, centroid(remaining[i])))
        assignment[ns] = remaining.pop(best_i)
    return assignment
