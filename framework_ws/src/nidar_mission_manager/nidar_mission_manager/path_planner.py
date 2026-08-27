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
# hd_camera as configured in this project: as2_gazebo_assets/models/hd_camera
# declares <horizontal_fov>1.0472</horizontal_fov> (60 deg) at 640x480.
#
# Only the ASPECT RATIO matters to the maths below, not the pixel count:
# swath width comes from the field of view and the altitude, and a metre of
# ground is a metre of ground however many pixels cover it. 640x480 is the
# same 4:3 as the original 1280x960, so the resolution reduction made for
# simulator throughput changed none of these numbers. Update these two
# constants if the camera's aspect ratio ever changes.
DEFAULT_CAMERA_HFOV_DEG = 60.0
DEFAULT_CAMERA_IMAGE_WIDTH_PX = 640
DEFAULT_CAMERA_IMAGE_HEIGHT_PX = 480
DEFAULT_OVERLAP_PCT = 20.0


def camera_vfov_deg(hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG,
                     image_width_px: int = DEFAULT_CAMERA_IMAGE_WIDTH_PX,
                     image_height_px: int = DEFAULT_CAMERA_IMAGE_HEIGHT_PX) -> float:
    """Vertical FOV implied by the horizontal FOV and the image aspect ratio.

    A pinhole camera's two FOVs are linked through the sensor aspect, not
    equal: 60 deg horizontal at 4:3 gives ~46.8 deg vertical.
    """
    aspect = image_width_px / image_height_px
    return math.degrees(2.0 * math.atan(math.tan(math.radians(hfov_deg) / 2.0) / aspect))


def ground_footprint_m(altitude_m: float,
                        hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG,
                        image_width_px: int = DEFAULT_CAMERA_IMAGE_WIDTH_PX,
                        image_height_px: int = DEFAULT_CAMERA_IMAGE_HEIGHT_PX
                        ) -> Tuple[float, float]:
    """Ground area one nadir frame covers at altitude_m: (long_m, short_m).

    e.g. 10m altitude, 60 deg HFOV, 4:3 -> (11.55, 8.66) -- ~100 m^2.
    """
    vfov = camera_vfov_deg(hfov_deg, image_width_px, image_height_px)
    long_m = 2.0 * altitude_m * math.tan(math.radians(hfov_deg) / 2.0)
    short_m = 2.0 * altitude_m * math.tan(math.radians(vfov) / 2.0)
    return long_m, short_m


def swath_width_m(altitude_m: float,
                   hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG,
                   image_width_px: int = DEFAULT_CAMERA_IMAGE_WIDTH_PX,
                   image_height_px: int = DEFAULT_CAMERA_IMAGE_HEIGHT_PX,
                   conservative: bool = True) -> float:
    """Cross-track ground width one pass reliably covers.

    conservative=True (default) uses the SHORT footprint edge. The frame is
    rectangular (11.55m x 8.66m at 10m), so which edge lies across-track
    depends on the drone's yaw relative to the flight leg -- and AS2's
    follow_path keeps yaw fixed (YawMode.KEEP_YAW) rather than turning it
    into each leg, so that alignment is not guaranteed. Planning on the wide
    edge leaves real gaps whenever the camera happens to be rotated; the
    short edge covers regardless of yaw. It also yields tighter line spacing
    and therefore a denser, genuinely serpentine path instead of two long
    parallel lines.
    """
    long_m, short_m = ground_footprint_m(altitude_m, hfov_deg, image_width_px, image_height_px)
    return short_m if conservative else long_m


def generate_lawnmower_path(zone_latlon: List[Point],
                             origin_lat: float, origin_lon: float, origin_alt: float = 0.0,
                             scan_altitude_m: float = DEFAULT_SCAN_ALTITUDE_M,
                             camera_hfov_deg: float = DEFAULT_CAMERA_HFOV_DEG,
                             overlap_pct: float = DEFAULT_OVERLAP_PCT,
                             flight_line_heading_deg: Optional[float] = None,
                             image_width_px: int = DEFAULT_CAMERA_IMAGE_WIDTH_PX,
                             image_height_px: int = DEFAULT_CAMERA_IMAGE_HEIGHT_PX,
                             start_near_latlon: Optional[Point] = None
                             ) -> List[Tuple[float, float, float]]:
    """Returns ordered [(lat, lon, alt_m), ...] waypoints covering zone_latlon.

    start_near_latlon: if given (typically the drone's own launch position),
    the serpentine is oriented to begin at the zone corner nearest that
    point, so the drone starts scanning as soon as it arrives instead of
    dead-heading across its zone first.
    """
    if len(zone_latlon) < 3:
        raise ValueError('zone must have at least 3 vertices')
    if not (0.0 <= overlap_pct < 100.0):
        raise ValueError('overlap_pct must be in [0, 100)')

    zone_enu = geo_utils.latlon_to_enu(zone_latlon, origin_lat, origin_lon, origin_alt)

    # Cross-track swath from the real camera geometry (see swath_width_m):
    # the SHORT footprint edge, so coverage holds regardless of the drone's
    # yaw on a leg. Was previously the HFOV/wide edge, which both assumed an
    # alignment AS2 doesn't guarantee and produced far too few lines.
    swath_width = swath_width_m(scan_altitude_m, camera_hfov_deg,
                                 image_width_px, image_height_px)
    line_spacing = swath_width * (1.0 - overlap_pct / 100.0)
    if line_spacing <= 0:
        raise ValueError('computed line_spacing <= 0 (check overlap_pct/hfov/altitude)')

    heading_deg = flight_line_heading_deg
    if heading_deg is None:
        heading_deg = _default_heading_deg(zone_enu)
    heading_rad = math.radians(heading_deg)

    # Rotate so the desired flight-line direction becomes the local +x axis.
    rotated = _rotate(zone_enu, -heading_rad)
    start_rot = None
    if start_near_latlon is not None:
        start_enu = geo_utils.latlon_to_enu([start_near_latlon], origin_lat, origin_lon, origin_alt)
        start_rot = _rotate(start_enu, -heading_rad)[0]
    waypoints_rot = _generate_scan_lines(rotated, line_spacing, start_rot)
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


def _generate_scan_lines(polygon: List[Point], line_spacing: float,
                          start_near: Optional[Point] = None) -> List[Point]:
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

    # Begin at whichever corner of the zone is nearest the drone, so it does
    # not transit the length of its own zone before starting to scan. Two
    # independent choices, both preserving a valid boustrophedon:
    #   - which end of the y range the first line sits at (reverse y_levels)
    #   - which x direction that first line runs in (flip the parity)
    flip_x = False
    if start_near is not None:
        sx, sy = start_near
        if abs(sy - max_y) < abs(sy - min_y):
            y_levels = list(reversed(y_levels))
        xs_all = [p[0] for p in polygon]
        min_x, max_x = min(xs_all), max(xs_all)
        # Run the first line away from the drone's side, i.e. start at the
        # near end: if the drone is nearer max_x, the first pass should go
        # max_x -> min_x.
        flip_x = abs(sx - max_x) < abs(sx - min_x)

    waypoints: List[Point] = []
    for line_idx, y in enumerate(y_levels):
        xs = _scanline_intersections(polygon, y)
        if len(xs) < 2:
            continue
        if len(xs) % 2 != 0:
            xs = xs[:-1]  # drop a degenerate single-vertex touch
        segments = [(xs[j], xs[j + 1]) for j in range(0, len(xs), 2)]
        if (line_idx % 2 == 1) != flip_x:            # boustrophedon: alternate
            segments = [(b, a) for a, b in reversed(segments)]
        for x_a, x_b in segments:
            waypoints.append((x_a, y))
            waypoints.append((x_b, y))
    return waypoints
