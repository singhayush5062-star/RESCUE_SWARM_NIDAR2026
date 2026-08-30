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

  return calculateCentroid(polygon);
}

/**
 * Generates a random realistic convex polygon mapping area around a center point.
 */
export function generateRandomBoundaryArea(
  center: [number, number] = [28.682412, 77.499734],
  radiusMeters: number = 60,
): [number, number][] {
  const [centerLat, centerLon] = center;
  const cosLat = Math.cos((centerLat * Math.PI) / 180.0);

  const numVertices = 5 + Math.floor(Math.random() * 2); // 5 or 6 vertices
  const angles: number[] = [];

  for (let i = 0; i < numVertices; i++) {
    angles.push((i * (2 * Math.PI / numVertices)) + (Math.random() * 0.3 - 0.15));
  }

  angles.sort((a, b) => a - b);

  return angles.map((angle) => {
    const r = radiusMeters * (0.75 + Math.random() * 0.5); // slight variation in radius
    const dLat = (r * Math.cos(angle)) / METERS_PER_DEGREE_LAT;
    const dLon = (r * Math.sin(angle)) / (METERS_PER_DEGREE_LAT * cosLat);
    return [centerLat + dLat, centerLon + dLon] as [number, number];
  });
}

/**
 * Approximate area in square meters of a [lat, lon] polygon.
 */
export function calculatePolygonAreaM2(polygon: [number, number][]): number {
  if (!polygon || polygon.length < 3) return 0;
  const cosLat = Math.cos((calculateCentroid(polygon)[0] * Math.PI) / 180.0);
  const points = polygon.map(([lat, lon]) => [
    lon * METERS_PER_DEGREE_LAT * cosLat,
    lat * METERS_PER_DEGREE_LAT,
  ]);
  let sum = 0;
  for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
    sum += points[j][0] * points[i][1] - points[i][0] * points[j][1];
  }
  return Math.abs(sum) / 2.0;
}

function approxDistanceMeters(a: [number, number], b: [number, number]): number {
  const cosLat = Math.cos((((a[0] + b[0]) / 2) * Math.PI) / 180.0);
  const dLat = (a[0] - b[0]) * METERS_PER_DEGREE_LAT;
  const dLon = (a[1] - b[1]) * METERS_PER_DEGREE_LAT * cosLat;
  return Math.sqrt(dLat * dLat + dLon * dLon);
}

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

const FEET_TO_METERS = 0.3048;
export const LAUNCH_BOX_SIZE_FT = 12;
export const LAUNCH_BOX_SIZE_M = LAUNCH_BOX_SIZE_FT * FEET_TO_METERS;

const DEFAULT_DRONE_OFFSETS: Record<string, [number, number]> = {
  drone0: [-1.4, -1.4],
  drone1: [1.4, -1.4],
  drone2: [-1.4, 1.4],
  drone3: [1.4, 1.4],
};

export function launchBoxCorners(center: [number, number]): [number, number][] {
  const half = LAUNCH_BOX_SIZE_M / 2;
  const cosLat = Math.cos((center[0] * Math.PI) / 180.0);
  const dLat = half / METERS_PER_DEGREE_LAT;
  const dLon = half / (METERS_PER_DEGREE_LAT * cosLat);
  const [lat, lon] = center;
  return [
    [lat - dLat, lon - dLon],
    [lat - dLat, lon + dLon],
    [lat + dLat, lon + dLon],
    [lat + dLat, lon - dLon],
  ];
}

export function defaultDroneLaunchPositions(
  center: [number, number],
  droneNamespaces: string[] = ['drone0', 'drone1', 'drone2', 'drone3'],
): Record<string, [number, number]> {
  const cosLat = Math.cos((center[0] * Math.PI) / 180.0);
  const result: Record<string, [number, number]> = {};
  droneNamespaces.forEach((ns, i) => {
    const [dx, dy] = DEFAULT_DRONE_OFFSETS[ns] ?? [((i % 2) * 2 - 1) * 1.4, (Math.floor(i / 2) * 2 - 1) * 1.4];
    result[ns] = [
      center[0] + dy / METERS_PER_DEGREE_LAT,
      center[1] + dx / (METERS_PER_DEGREE_LAT * cosLat),
    ];
  });
  return result;
}

export function generateRandomDroneLaunchPositions(
  center: [number, number],
  droneNamespaces: string[] = ['drone0', 'drone1', 'drone2', 'drone3'],
  minSeparationM: number = 1.0,
): Record<string, [number, number]> {
  const points = generateRandomPointsInPolygon(launchBoxCorners(center), droneNamespaces.length, minSeparationM);
  const result: Record<string, [number, number]> = {};
  droneNamespaces.forEach((ns, i) => {
    if (points[i]) result[ns] = points[i];
  });
  return result;
}

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
    const offset = DEFAULT_DRONE_OFFSETS[ns] || [((i % 2) * 2 - 1) * 1.4, Math.floor(i / 2) * 1.4];

    const dLon = offset[0] / metersPerDegreeLon;
    const dLat = offset[1] / METERS_PER_DEGREE_LAT;

    result[ns] = [[homeLat + dLat, homeLon + dLon]];
  }

  return result;
}

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
