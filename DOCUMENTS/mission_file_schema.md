# NIDAR mission file schema (placeholder)

The organisers do not release their boundary/mission file format until competition
setup time (Mission Brief §3), so this is a **placeholder schema we control** for
Phase 1 development. The loader (`gcs/src/mission/parseMissionFile.ts` +
`project_gazebo/mission_file_executor.py`) is the only place that understands this
shape — when the real format shows up, only that parsing layer needs to change,
not the GCS UI or the flight-execution logic.

## Format: JSON

```json
{
  "mission_name": "search_area_1",
  "boundary": [
    [40.440500, -3.689900],
    [40.440700, -3.689900],
    [40.440700, -3.689600],
    [40.440500, -3.689600]
  ],
  "home": [40.440529, -3.689828],
  "altitude_m": 1.0,
  "speed_mps": 0.5,
  "drones": {
    "drone0": [
      [40.440600, -3.689850],
      [40.440650, -3.689750],
      [40.440550, -3.689700]
    ],
    "drone1": [
      [40.440450, -3.689850],
      [40.440400, -3.689750],
      [40.440500, -3.689700]
    ]
  }
}
```

- `boundary`: polygon (lat, lon) defining the outer search-area edge. Display-only in
  Phase 1 — the coverage/area-division planner that actually splits this polygon into
  per-drone search patterns is Phase 6 scope, not Phase 1.
- `home`: launch/landing point, must fall inside the 12ft x 12ft pad per the mission brief.
- `altitude_m` / `speed_mps`: shared defaults applied to every waypoint.
- `drones`: explicit per-namespace waypoint lists. This is what Phase 1 actually flies —
  a fixed, file-defined path per drone, satisfying "no manual waypoint entry."

## Why JSON, not the organiser's real format

Building against an unknown future format would mean guessing twice. Instead:
1. The GCS and executor only ever consume this internal schema.
2. `parseMissionFile.ts` is the single conversion point from an uploaded file to this
   schema — swapping in the real organiser format later means rewriting one function,
   not the mission-loading UI, the map rendering, or the flight executor.
