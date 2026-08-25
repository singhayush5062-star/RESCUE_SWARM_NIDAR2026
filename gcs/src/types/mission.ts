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
  drones?: Record<string, [number, number][]>;
}

export type MissionStatusState =
  | 'idle'
  | 'loaded'
  | 'starting'
  | 'taking_off'
  | 'running'
  | 'landing'
  | 'complete'
  | 'error';

export interface MissionStatus {
  state: MissionStatusState;
  detail: string;
  drones?: string[];
  waypoint_index?: number;
}
