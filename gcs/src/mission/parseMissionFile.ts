import type { MissionFile } from '../types/mission';
import { parseKmlFile } from './parseKml';

/**
 * The only place that understands our own JSON mission-file format. See
 * DOCUMENTS/mission_file_schema.md — this is a placeholder schema we control,
 * kept around for dev/testing since it's fully self-contained (altitude,
 * speed, and every drone's waypoints), unlike the organisers' KML which only
 * defines the arena boundary (see parseKml.ts).
 */
function parseJsonMissionFile(raw: unknown): MissionFile {
  if (typeof raw !== 'object' || raw === null) {
    throw new Error('Mission file must be a JSON object');
  }
  const m = raw as Record<string, unknown>;

  if (typeof m.mission_name !== 'string') throw new Error('Missing "mission_name"');
  if (!Array.isArray(m.boundary) || m.boundary.length < 3) {
    throw new Error('"boundary" must be a polygon with at least 3 points');
  }
  if (!Array.isArray(m.home) || m.home.length !== 2) throw new Error('Missing "home" [lat, lon]');
  if (typeof m.altitude_m !== 'number') throw new Error('Missing "altitude_m"');
  if (typeof m.speed_mps !== 'number') throw new Error('Missing "speed_mps"');
  if (typeof m.drones !== 'object' || m.drones === null || Object.keys(m.drones).length === 0) {
    throw new Error('"drones" must map at least one namespace to a waypoint list');
  }

  for (const [ns, wps] of Object.entries(m.drones as Record<string, unknown>)) {
    if (!Array.isArray(wps) || wps.some((wp) => !Array.isArray(wp) || wp.length !== 2)) {
      throw new Error(`Drone "${ns}" waypoints must be a list of [lat, lon] pairs`);
    }
  }

  return { ...m, source: 'json' } as unknown as MissionFile;
}

/** Dispatches on file extension: `.kml` for the organisers' arena boundary
 * file, `.json` for our own dev/testing schema. Both converge on the same
 * MissionFile shape the rest of the GCS (map, status panel, mission control)
 * already works with. */
export async function loadMissionFileFromInput(file: File): Promise<MissionFile> {
  if (file.name.toLowerCase().endsWith('.kml')) {
    return parseKmlFile(file);
  }

  const text = await file.text();
  let parsed: unknown;
  try {
    parsed = JSON.parse(text);
  } catch {
    throw new Error('File is not valid JSON');
  }
  return parseJsonMissionFile(parsed);
}
