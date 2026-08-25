"""Boustrophedon (lawnmower) coverage path generation for one zone polygon.

Flight-line heading convention: `flight_line_heading_deg` is the compass-free,
math-standard angle (degrees, CCW from local ENU +x/East) of the direction
individual flight LINES travel in. If not given, it is auto-computed from the
zone's own axis-aligned ENU bounding box: lines run parallel to the box's
LONGER side, and are stepped/spaced across its SHORTER side -- e.g. a 100m x
20m zone flown along its 100m axis needs only ~4 lines; flown the other way
it would need ~5x as many, much shorter, lines. NOT a full rotating-calipers
minimum-bounding-rectangle computation -- axis-aligned bbox is sufficient
because zone_splitter's own output zones are already axis-aligned strips of
the (in practice rectangular) arena.
"""
import math
from typing import List, Optional, Tuple

from . import geo_utils

Point = Tuple[float, float]

DEFAULT_SCAN_ALTITUDE_M = 25.0
DEFAULT_CAMERA_HFOV_DEG = 60.0
DEFAULT_OVERLAP_PCT = 20.0


def generate_lawnmower_path(zone_latlon: List[Point],
                             origin_lat: float, origin_lon: float, origin_alt: float = 0.0,
                             scan_altitude_m: float = DEFAULT_SCAN_ALTITUDE_M,
                             camera_hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG,
                             overlap_pct: float = DEFAULT_OVERLAP_PCT,
                             flight_line_heading_deg: Optional[float] = None
                             ) -> List[Tuple[float, float, float]]:
    """Returns ordered [(lat, lon, alt_m), ...] waypoints covering zone_latlon."""
    if len(zone_latlon) < 3:
        raise ValueError('zone must have at least 3 vertices')
    if not (0.0 <= overlap_pct < 100.0):
        raise ValueError('overlap_pct must be in [0, 100)')

    zone_enu = geo_utils.latlon_to_enu(zone_latlon, origin_lat, origin_lon, origin_alt)

    swath_width = 2.0 * scan_altitude_m * math.tan(math.radians(camera_hfov_deg) / 2.0)
    line_spacing = swath_width * (1.0 - overlap_pct / 100.0)
    if line_spacing <= 0:
        raise ValueError('computed line_spacing <= 0 (check overlap_pct/hfov/altitude)')

    heading_deg = flight_line_heading_deg
    if heading_deg is None:
        heading_deg = _default_heading_deg(zone_enu)
    heading_rad = math.radians(heading_deg)

    # Rotate so the desired flight-line direction becomes the local +x axis.
    rotated = _rotate(zone_enu, -heading_rad)
    waypoints_rot = _generate_scan_lines(rotated, line_spacing)
    waypoints_enu = _rotate(waypoints_rot, heading_rad)

    latlon_2d = geo_utils.enu_to_latlon(waypoints_enu, origin_lat, origin_lon, origin_alt)
    return [(lat, lon, scan_altitude_m) for lat, lon in latlon_2d]


def _default_heading_deg(polygon_enu: List[Point]) -> float:
    xs = [p[0] for p in polygon_enu]; ys = [p[1] for p in polygon_enu]
    width, height = max(xs) - min(xs), max(ys) - min(ys)
    return 0.0 if width >= height else 90.0


def _rotate(points: List[Point], angle_rad: float) -> List[Point]:
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return [(x * c - y * s, x * s + y * c) for x, y in points]


def _scanline_intersections(polygon: List[Point], y: float) -> List[float]:
    """Ray-casting: x-coordinates where horizontal line y=const crosses the
    polygon's edges. Handles convex AND concave polygons -- multiple (enter,
    exit) pairs on one line are possible for concave zones."""
    xs = []
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if y1 == y2:
            continue
        if (y1 <= y < y2) or (y2 <= y < y1):
            t = (y - y1) / (y2 - y1)
            xs.append(x1 + t * (x2 - x1))
    xs.sort()
    return xs


def _generate_scan_lines(polygon: List[Point], line_spacing: float) -> List[Point]:
    ys = [p[1] for p in polygon]
    min_y, max_y = min(ys), max(ys)
    height = max_y - min_y

    n_lines = max(1, math.ceil(height / line_spacing))  # never 0: a zone
    # narrower than one swath still gets exactly one covering pass through
    # its own vertical center, rather than silently returning no waypoints.
    if n_lines == 1:
        y_levels = [(min_y + max_y) / 2.0]
    else:
        # Center the n_lines line-centers (spaced line_spacing apart) within
        # [min_y, max_y], rather than offsetting a fixed half-spacing from
        # min_y. When height isn't an exact multiple of line_spacing (the
        # common case, since n_lines is rounded up), (n_lines-1)*line_spacing
        # can be close to -- or even exceed -- height; offsetting from min_y
        # by a flat line_spacing/2 then overshoots max_y and drops whichever
        # lines land outside the polygon, silently undercounting coverage.
        span_used = (n_lines - 1) * line_spacing
        start_y = min_y + (height - span_used) / 2.0
        y_levels = [start_y + k * line_spacing for k in range(n_lines)]

    waypoints: List[Point] = []
    for line_idx, y in enumerate(y_levels):
        xs = _scanline_intersections(polygon, y)
        if len(xs) < 2:
            continue
        if len(xs) % 2 != 0:
            xs = xs[:-1]  # drop a degenerate single-vertex touch
        segments = [(xs[j], xs[j + 1]) for j in range(0, len(xs), 2)]
        if line_idx % 2 == 1:                       # boustrophedon: alternate
            segments = [(b, a) for a, b in reversed(segments)]
        for x_a, x_b in segments:
            waypoints.append((x_a, y))
            waypoints.append((x_b, y))
    return waypoints
