"""Splits a boundary polygon into N sub-polygons (one per drone) by
bounding-box strip slicing + Sutherland-Hodgman clipping.

Sutherland-Hodgman is exact here because every clip window this module
builds is an axis-aligned rectangle (convex) -- it correctly handles a
concave *subject* polygon (the boundary) even though it would not, in
general, handle a concave *clip* polygon.
"""
from typing import List, Tuple

from . import geo_utils

Point = Tuple[float, float]

_EPS = 1e-6           # generic float-comparison slack
_PAD_M = 5.0           # perpendicular padding on the clip rectangle, in meters,
                        # so a polygon vertex that lands exactly on the bbox's
                        # own edge (common for axis-aligned rectangular arenas)
                        # is never dropped by a strict inside/outside test


def split_boundary(boundary_latlon: List[Point], num_zones: int,
                    origin_lat: float, origin_lon: float, origin_alt: float = 0.0
                    ) -> List[List[Point]]:
    """Entry point. boundary_latlon: [(lat, lon), ...], polygon, >= 3 points.
    Returns num_zones sub-polygons, each [(lat, lon), ...], covering the
    original boundary with no gaps and (for convex/rectangular input) no
    overlap. Raises ValueError on bad input or a degenerate split result.
    """
    if num_zones < 1:
        raise ValueError('num_zones must be >= 1')
    if len(boundary_latlon) < 3:
        raise ValueError('boundary must have at least 3 vertices')

    boundary_enu = geo_utils.latlon_to_enu(boundary_latlon, origin_lat, origin_lon, origin_alt)
    zones_enu = _split_polygon_enu(boundary_enu, num_zones)
    return [geo_utils.enu_to_latlon(z, origin_lat, origin_lon, origin_alt) for z in zones_enu]


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
