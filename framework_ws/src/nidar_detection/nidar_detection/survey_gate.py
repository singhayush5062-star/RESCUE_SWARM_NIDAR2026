"""When is a drone actually surveying, and so worth running inference on?

Pure predicate, no ROS, so the rule can be tested against hand-written states
rather than by flying a mission.

The problem this solves, measured on a 15-survivor run: the swarm reported 23
survivors. Fourteen were real and located to 0.08-0.84 m. Six were phantoms,
and four of those sat within 5 m of the launch box centre -- one just 1.29 m
from it, essentially on a drone pad. During takeoff, climb, return and
landing every drone's nadir camera stares down at a 3.66 m box containing
three other quadrotors, and the person model scores them 0.90-0.96. They are
confident, correct-looking detections of things that are not people, and no
amount of downstream de-duplication can tell them from a real survivor,
because geographically they ARE a cluster of consistent sightings.

The fix is upstream of all of that: only look while actually mapping. A
survey pass is the only part of a mission whose imagery the detector was
trained for -- nadir, at scan altitude, over the search area.
"""

from typing import Optional

#: Mission state, as published by nidar_mission_executor on
#: /nidar/mission_status, during which the coverage pattern is being flown.
SURVEY_STATE = 'running'

#: Default half-window around the mission's scan altitude. Wide enough to
#: absorb the controller's own altitude hold (measured well inside a metre)
#: and the climb settling at the start of a leg, narrow enough to exclude the
#: staggered return-to-launch altitudes, which sit 2.5 m apart.
DEFAULT_ALTITUDE_TOLERANCE_M = 2.0


def is_surveying(mission_state: Optional[str],
                 altitude_m: Optional[float],
                 scan_altitude_m: Optional[float],
                 tolerance_m: float = DEFAULT_ALTITUDE_TOLERANCE_M) -> bool:
    """True when this drone is flying the survey pattern at survey height.

    Any input being None means "not known yet", which is NOT the same as
    "condition satisfied" -- a node that has not yet heard a mission status,
    or has no pose, must not infer. Returning True on unknown state is what
    would quietly restore the old always-on behaviour the first time a topic
    was slow to arrive.
    """
    if mission_state != SURVEY_STATE:
        return False
    if altitude_m is None or scan_altitude_m is None:
        return False
    return abs(altitude_m - scan_altitude_m) <= tolerance_m


def point_in_polygon(lat: float, lon: float, polygon) -> bool:
    """Ray-casting point-in-polygon on (lat, lon) vertices.

    Degrees are used directly rather than projecting to metres first. Over a
    zone a few tens of metres across the longitude scaling is a constant
    factor, and a constant scaling of one axis cannot change which side of an
    edge a point falls on -- so the test is exact for this purpose and needs
    no origin. Same approach the GCS already uses in launchSiteManager.ts.

    An empty or degenerate polygon returns False: "no zone known" must not
    read as "everywhere is my zone".
    """
    if not polygon or len(polygon) < 3:
        return False
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        lat_i, lon_i = polygon[i][0], polygon[i][1]
        lat_j, lon_j = polygon[j][0], polygon[j][1]
        if (lon_i > lon) != (lon_j > lon):
            if lat < (lat_j - lat_i) * (lon - lon_i) / (lon_j - lon_i) + lat_i:
                inside = not inside
        j = i
    return inside


#: Metres per degree of latitude, for the small-area approximations below.
METERS_PER_DEGREE_LAT = 111320.0


def _segment_distance_m(lat, lon, a, b) -> float:
    """Metres from (lat, lon) to the segment a-b, all in (lat, lon)."""
    import math
    scale = math.cos(math.radians(lat))
    px, py = lon * scale, lat
    ax, ay = a[1] * scale, a[0]
    bx, by = b[1] * scale, b[0]
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        t = 0.0
    else:
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy) * METERS_PER_DEGREE_LAT


def is_inside_area(lat: float, lon: float, polygon, margin_m: float = 0.0) -> bool:
    """Is (lat, lon) inside `polygon`, or within `margin_m` of its edge?

    The margin exists because the boundary is a hard edge applied to an
    estimated position. Measured geotag error on this system is 0.40 m mean
    and 1.16 m worst case, so a survivor standing just inside the line can
    project just outside it. Rejecting them would turn a position error into
    a missing person, which is the worse failure by far.

    An empty polygon returns True, not False: "no boundary known" must mean
    "do not filter", never "reject everything". Getting that backwards would
    silently discard every survivor on any mission that did not publish a
    boundary.
    """
    if not polygon or len(polygon) < 3:
        return True
    if point_in_polygon(lat, lon, polygon):
        return True
    if margin_m <= 0.0:
        return False
    n = len(polygon)
    return any(_segment_distance_m(lat, lon, polygon[i], polygon[(i + 1) % n]) <= margin_m
               for i in range(n))
