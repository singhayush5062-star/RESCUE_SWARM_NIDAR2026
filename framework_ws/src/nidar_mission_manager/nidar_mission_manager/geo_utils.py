"""Lat/lon <-> local ENU-meters conversion relative to a caller-supplied origin
(the same origin world_swarm.yaml's `origin:` block and AS2's state estimator use).
All geometry math elsewhere in this package works in ENU meters, never raw degrees.
"""
import pymap3d


def latlon_to_enu(points, origin_lat, origin_lon, origin_alt=0.0):
    """[(lat, lon), ...] -> [(east_m, north_m), ...]. Flat-terrain assumption
    (all points treated at origin_alt) -- fine for a <1km survey with no elevation data."""
    return [pymap3d.geodetic2enu(lat, lon, origin_alt, origin_lat, origin_lon, origin_alt, deg=True)[:2]
            for lat, lon in points]


def enu_to_latlon(points, origin_lat, origin_lon, origin_alt=0.0):
    """Inverse of latlon_to_enu."""
    return [pymap3d.enu2geodetic(e, n, 0.0, origin_lat, origin_lon, origin_alt, deg=True)[:2]
            for e, n in points]
