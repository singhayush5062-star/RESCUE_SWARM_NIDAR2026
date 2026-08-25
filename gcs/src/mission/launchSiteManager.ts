import type { MissionFile } from '../types/mission';

/**
 * Standard WGS-84 meters-per-degree approximation at equator.
 */
const METERS_PER_DEGREE_LAT = 111320.0;

/**
 * Checks if a [lat, lon] point lies strictly inside a polygon boundary defined by [lat, lon] vertices.
 * Uses the Ray-Casting algorithm.
 */
export function isPointInPolygon(point: [number, number], polygon: [number, number][]): boolean {
  if (!polygon || polygon.length < 3) return false;
  const [x, y] = point; // lat, lon
  let inside = false;

  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i][0];
    const yi = polygon[i][1];
    const xj = polygon[j][0];
    const yj = polygon[j][1];

    const intersect = yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi;
    if (intersect) inside = !inside;
  }
  return inside;
}

/**
 * Computes the geometric centroid of a polygon boundary.
 */
export function calculateCentroid(polygon: [number, number][]): [number, number] {
  if (!polygon || polygon.length === 0) return [0, 0];
  let latSum = 0;
  let lonSum = 0;
  for (const [lat, lon] of polygon) {
    latSum += lat;
    lonSum += lon;
  }
  return [latSum / polygon.length, lonSum / polygon.length];
}

/**
 * Computes the bounding box [minLat, maxLat, minLon, maxLon] of a polygon.
 */
export function getBoundingBox(polygon: [number, number][]): [number, number, number, number] {
  let minLat = Infinity;
  let maxLat = -Infinity;
  let minLon = Infinity;
  let maxLon = -Infinity;

  for (const [lat, lon] of polygon) {
    if (lat < minLat) minLat = lat;
    if (lat > maxLat) maxLat = lat;
    if (lon < minLon) minLon = lon;
    if (lon > maxLon) maxLon = lon;
  }

  return [minLat, maxLat, minLon, maxLon];
}

/**
 * Generates a random launch site [lat, lon] coordinate located inside the KML boundary polygon.
 */
export function generateRandomLaunchSiteInKml(polygon: [number, number][]): [number, number] {
  if (!polygon || polygon.length < 3) return [0, 0];

  const [minLat, maxLat, minLon, maxLon] = getBoundingBox(polygon);
  let attempts = 0;

  while (attempts < 1000) {
    attempts++;
    const randomLat = minLat + Math.random() * (maxLat - minLat);
    const randomLon = minLon + Math.random() * (maxLon - minLon);

    if (isPointInPolygon([randomLat, randomLon], polygon)) {
      return [randomLat, randomLon];
    }
  }

  // Fallback to centroid if polygon shape is extremely irregular
  return calculateCentroid(polygon);
}

/**
 * Approximate great-circle-ish distance in meters between two [lat, lon] points,
 * accurate enough at the scale of a survey arena (equirectangular approximation,
 * same METERS_PER_DEGREE_LAT constant used throughout this module).
 */
function approxDistanceMeters(a: [number, number], b: [number, number]): number {
  const cosLat = Math.cos((((a[0] + b[0]) / 2) * Math.PI) / 180.0);
  const dLat = (a[0] - b[0]) * METERS_PER_DEGREE_LAT;
  const dLon = (a[1] - b[1]) * METERS_PER_DEGREE_LAT * cosLat;
  return Math.sqrt(dLat * dLat + dLon * dLon);
}

/**
 * Generates `count` random [lat, lon] points strictly inside the boundary
 * polygon, each at least `minSeparationM` apart from every other point already
 * picked in this batch (so a random batch doesn't stack dummies on top of each
 * other). Same bounded-attempts shape as generateRandomLaunchSiteInKml; falls
 * short of `count` (returns however many it found) rather than looping forever
 * on a polygon too small to fit them all with the requested separation.
 */
export function generateRandomPointsInPolygon(
  polygon: [number, number][],
  count: number,
  minSeparationM: number = 3.0,
): [number, number][] {
  if (!polygon || polygon.length < 3 || count <= 0) return [];

  const [minLat, maxLat, minLon, maxLon] = getBoundingBox(polygon);
  const points: [number, number][] = [];
  const maxAttempts = count * 500;
  let attempts = 0;

  while (points.length < count && attempts < maxAttempts) {
    attempts++;
    const candidate: [number, number] = [
      minLat + Math.random() * (maxLat - minLat),
      minLon + Math.random() * (maxLon - minLon),
    ];
    if (!isPointInPolygon(candidate, polygon)) continue;
    if (points.some((p) => approxDistanceMeters(p, candidate) < minSeparationM)) continue;
    points.push(candidate);
  }

  return points;
}

/**
 * Default starting relative offsets (in meters [dx, dy]) for drones in launch formation around Home.
 * drone0: (-2m, 0m)
 * drone1: (2m, 0m)
 * drone2: (0m, 0m)
 * drone3: (0m, 2m)
 */
const DEFAULT_DRONE_OFFSETS: Record<string, [number, number]> = {
  drone0: [-2.0, 0.0],
  drone1: [2.0, 0.0],
  drone2: [0.0, 0.0],
  drone3: [0.0, 2.0],
};

/**
 * Calculates GPS waypoints for a list of drone namespaces centered around a Home Launch Site [homeLat, homeLon].
 */
export function generateDroneWaypointsAtLaunchSite(
  homeLat: number,
  homeLon: number,
  droneNamespaces: string[] = ['drone0', 'drone1', 'drone2', 'drone3'],
): Record<string, [number, number][]> {
  const cosLat = Math.cos((homeLat * Math.PI) / 180.0);
  const metersPerDegreeLon = METERS_PER_DEGREE_LAT * cosLat;

  const result: Record<string, [number, number][]> = {};

  for (let i = 0; i < droneNamespaces.length; i++) {
    const ns = droneNamespaces[i];
    const offset = DEFAULT_DRONE_OFFSETS[ns] || [((i % 2) * 2 - 1) * 2.0, Math.floor(i / 2) * 2.0];

    const dLon = offset[0] / metersPerDegreeLon;
    const dLat = offset[1] / METERS_PER_DEGREE_LAT;

    result[ns] = [[homeLat + dLat, homeLon + dLon]];
  }

  return result;
}

/**
 * Helper to update or construct a complete MissionFile object with a specified home launch site and boundary.
 *
 * Deliberately does NOT set `drones` here. mission_file_executor.py dispatches
 * on whether `mission.drones` is populated: non-empty means "fly exactly
 * these pre-supplied per-drone waypoints" (the old sequential-GoTo flow),
 * while boundary-only (no `drones`) means "auto zone-split the boundary and
 * fly a generated lawnmower coverage pattern" (the Phase 1 flow this feature
 * is actually meant to trigger). generateDroneWaypointsAtLaunchSite's output
 * is single hover points, not a flight plan -- it's only ever meant as a
 * launch-formation *preview* (MapView.tsx renders it separately, straight
 * from mission.home, without reading mission.drones at all). Setting it here
 * previously made every GUI-drawn/launch-site mission silently skip the
 * coverage flow entirely and just hop each drone to its formation offset.
 */
export function createMissionFromLaunchSiteAndBoundary(
  boundary: [number, number][],
  home: [number, number],
  missionName: string = 'Interactive Mapping Mission',
  altitude_m: number = 25,
  speed_mps: number = 2.0,
): MissionFile {
  return {
    source: 'json',
    mission_name: missionName,
    boundary,
    home,
    altitude_m,
    speed_mps,
  };
}
