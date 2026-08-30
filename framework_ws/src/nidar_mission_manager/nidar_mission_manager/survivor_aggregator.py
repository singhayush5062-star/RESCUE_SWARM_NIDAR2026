"""Survivor identity: deciding when two sightings are the same person.

Zero ROS dependencies, like the rest of this package, so the merge rules can
be tested against hand-written coordinates with no simulator running.

One registry serves both dedup stages of the geotag pipeline, at different
radii:

  * **Per drone**, inside nidar_geotag's geotag_node -- a single drone sees
    one survivor in dozens of consecutive frames. Each frame is a separate
    DetectionResult and would otherwise be a separate survivor.
  * **Across drones**, inside survivor_aggregator_node -- two drones flying
    adjacent zones both see a survivor near their shared boundary. Their
    ByteTrack ids are unrelated (track_id scope is one drone, see
    DetectionResult.msg), so only geography can reconcile them.

Same rule, two radii, one implementation.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

#: Metres per degree of latitude. Longitude is scaled by cos(lat) on top of
#: this. Survivor separation is judged over metres, not hundreds of km, so
#: the equirectangular approximation is far below the error of the detection
#: itself -- this is the same approximation mission_executor uses for its
#: launch-box checks.
METERS_PER_DEGREE_LAT = 111320.0

# Dedup radii. These are a two-sided constraint, and both sides were
# measured on this arena rather than assumed:
#
#   LOWER BOUND -- must exceed the geotag position error, or one person seen
#   from two angles becomes two survivors. Measured per-view error on a full
#   coverage flight: 0.18 - 1.15 m.
#
#   UPPER BOUND -- must stay below the closest spacing between two genuinely
#   distinct survivors, or two people become one. The arena's closest pair is
#   3.03 m apart, and the GCS's own random survivor placement enforces
#   minSeparationM = 3.0, so 3 m is this project's real floor.
#
# DOCUMENTS/NIDAR_Implementation_Plan.md proposes 3.0 / 5.0 for these. That
# is a deliberate deviation, because 5.0 m was measured actively losing
# survivors: on a 14-survivor run it merged away FIVE of them --
# survivor_1 (3.03 m from survivor_2), survivor_8 (3.91 m from survivor_9),
# survivor_4 (4.29 m), survivor_5 (4.42 m) and survivor_12 (4.80 m) -- every
# one of them a distinct person standing inside the radius of a neighbour.
# The same over-merging also produced the run's two worst position errors
# (2.05 m and 1.15 m), because a merged record sits between the two people it
# conflated. Tightening the radius improves recall AND accuracy together.
#
# A survivor missed is a survivor not rescued, and the competition scores
# per survivor found, so the asymmetry favours the tighter radius: a
# duplicate is visible on the map and can be judged by an operator, whereas
# a merged-away survivor leaves no trace at all.
DEFAULT_LOCAL_DEDUP_RADIUS_M = 2.0
DEFAULT_GLOBAL_DEDUP_RADIUS_M = 2.5


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Ground distance in metres between two lat/lon pairs."""
    dlat = (lat2 - lat1) * METERS_PER_DEGREE_LAT
    dlon = ((lon2 - lon1) * METERS_PER_DEGREE_LAT
            * math.cos(math.radians((lat1 + lat2) / 2.0)))
    return math.hypot(dlat, dlon)


@dataclass
class SurvivorRecord:
    """One physical person, as currently believed."""

    survivor_id: int
    latitude: float
    longitude: float
    altitude: float
    #: Highest confidence any single sighting reported. Deliberately the max
    #: and not a mean: "how sure are we a person is here" is answered by the
    #: best look anyone got, not degraded by every poor-angle glimpse.
    confidence: float
    detecting_drones: List[str] = field(default_factory=list)
    observations: int = 0
    #: Merge keys already folded into this record (e.g. 'drone0:7'), so a
    #: track that wanders past the radius still lands on its own record.
    keys: List[str] = field(default_factory=list)
    #: Sum of confidences, the denominator of the weighted position mean.
    _weight: float = 0.0

    def as_dict(self) -> dict:
        return {
            'survivor_id': self.survivor_id,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
            'confidence': self.confidence,
            'detecting_drones': list(self.detecting_drones),
            'observations': self.observations,
        }


class SurvivorRegistry:
    """Accumulates sightings into a deduplicated survivor list."""

    def __init__(self,
                 dedup_radius_m: float = DEFAULT_GLOBAL_DEDUP_RADIUS_M,
                 id_start: int = 0):
        self.dedup_radius_m = float(dedup_radius_m)
        self._next_id = int(id_start)
        self._records: List[SurvivorRecord] = []

    @property
    def records(self) -> List[SurvivorRecord]:
        return list(self._records)

    def __len__(self) -> int:
        return len(self._records)

    def find(self, survivor_id: int) -> Optional[SurvivorRecord]:
        for r in self._records:
            if r.survivor_id == survivor_id:
                return r
        return None

    def _match(self, lat: float, lon: float,
                key: Optional[str]) -> Optional[SurvivorRecord]:
        # An explicit key wins over geometry. A ByteTrack id is a far stronger
        # statement than "these two points are close": it means the tracker
        # followed this exact person from frame to frame. Trusting distance
        # first would split one track that drifted past the radius, and merge
        # two people standing within it.
        if key is not None:
            for r in self._records:
                if key in r.keys:
                    return r
        best, best_d = None, self.dedup_radius_m
        for r in self._records:
            d = haversine_m(lat, lon, r.latitude, r.longitude)
            if d <= best_d:
                best, best_d = r, d
        return best

    def observe(self, latitude: float, longitude: float, altitude: float,
                confidence: float, drone_id: str,
                key: Optional[str] = None) -> Tuple[SurvivorRecord, bool]:
        """Fold one sighting in. Returns (record, is_new).

        Position is a confidence-weighted running mean rather than the latest
        fix. A single frame's geotag carries the full bbox jitter of that one
        frame; averaging tens of sightings is what gets the position inside
        the accuracy target, and weighting by confidence keeps a marginal
        glimpse from dragging a well-observed survivor off its spot.
        """
        confidence = float(confidence)
        record = self._match(latitude, longitude, key)

        if record is None:
            record = SurvivorRecord(
                survivor_id=self._next_id,
                latitude=float(latitude), longitude=float(longitude),
                altitude=float(altitude), confidence=confidence,
                detecting_drones=[drone_id], observations=1,
                keys=[key] if key is not None else [],
                _weight=max(confidence, 1e-6),
            )
            self._next_id += 1
            self._records.append(record)
            return record, True

        # Guard the degenerate case where every sighting so far scored 0.0:
        # a zero denominator would make the mean NaN and silently destroy the
        # record's position.
        w = max(confidence, 1e-6)
        total = record._weight + w
        record.latitude = (record.latitude * record._weight + latitude * w) / total
        record.longitude = (record.longitude * record._weight + longitude * w) / total
        record.altitude = (record.altitude * record._weight + altitude * w) / total
        record._weight = total
        record.confidence = max(record.confidence, confidence)
        record.observations += 1
        if drone_id not in record.detecting_drones:
            record.detecting_drones.append(drone_id)
        if key is not None and key not in record.keys:
            record.keys.append(key)
        return record, False
