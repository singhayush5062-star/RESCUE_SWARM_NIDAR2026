/**
 * A loaded mission. `source: 'json'` missions (our own placeholder schema,
 * see DOCUMENTS/mission_file_schema.md) are always fully specified — altitude,
 * speed, and every drone's waypoints. `source: 'kml'` missions come from the
 * organisers' arena boundary file, which per their format only defines the
 * outer fence — no altitude/speed/waypoints. Those get filled in later by our
 * own mapping/coverage algorithm (not yet integrated); until then `drones` is
 * absent and the mission can be viewed but not started.
 */
export interface MissionFile {
  source: 'json' | 'kml';
  mission_name: string;
  boundary: [number, number][];
  home: [number, number];
  altitude_m?: number;
  speed_mps?: number;
  /** Per-drone flight-altitude override (namespace -> meters), falling back
   * to altitude_m for any drone not listed here. */
  drone_altitudes?: Record<string, number>;
  /** Per-drone launch position (namespace -> [lat, lon]) within the fixed
   * 12ft x 12ft launch/landing box centered on `home`. Any drone not listed
   * launches from its default world-config position. The backend teleports
   * each listed drone there before arming (no sim restart) and rejects the
   * mission if a position falls outside the box. */
  drone_launch_positions?: Record<string, [number, number]>;
  drones?: Record<string, [number, number][]>;
}

export type MissionStatusState =
  | 'idle'
  | 'loaded'
  | 'starting'
  | 'taking_off'
  | 'running'
  | 'returning'
  | 'landing'
  | 'complete'
  | 'error';

export interface MissionStatus {
  state: MissionStatusState;
  detail: string;
  drones?: string[];
  waypoint_index?: number;
}
