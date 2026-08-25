import { kml } from '@tmcw/togeojson';
import type { Feature, Polygon, MultiPolygon } from 'geojson';
import type { MissionFile } from '../types/mission';

/**
 * The organisers' arena file only defines the outer boundary fence (no home
 * point, no per-drone paths) — see the conversation that scoped this down
 * from DOCUMENTS's original placeholder-schema assumption. Our own
 * mapping/coverage algorithm (not yet integrated) is what will turn this
 * boundary into flyable waypoints later.
 *
 * KML/GeoJSON coordinates are always [lon, lat] — the opposite of this
 * project's [lat, lon] convention used everywhere else (mission JSON schema,
 * GPS topics, DroneInterfaceGPS.go_to_gps_point). The swap happens exactly
 * once, here, so nothing downstream has to think about it.
 */
export async function parseKmlFile(file: File): Promise<MissionFile> {
  const text = await file.text();
  const xml = new DOMParser().parseFromString(text, 'text/xml');

  const parserError = xml.querySelector('parsererror');
  if (parserError) {
    throw new Error('File is not valid XML/KML');
  }

  const geojson = kml(xml);
  const polygonFeature = geojson.features.find(
    (f): f is Feature<Polygon | MultiPolygon> => f.geometry?.type === 'Polygon' || f.geometry?.type === 'MultiPolygon',
  );
  if (!polygonFeature) {
    throw new Error('No boundary polygon found in this KML file');
  }

  const ring =
    polygonFeature.geometry.type === 'Polygon'
      ? polygonFeature.geometry.coordinates[0]
      : polygonFeature.geometry.coordinates[0][0];

  const boundary: [number, number][] = ring.map(([lon, lat]) => [lat, lon]);
  const home = centroid(boundary);

  return {
    source: 'kml',
    mission_name: polygonFeature.properties?.name ?? file.name.replace(/\.kml$/i, ''),
    boundary,
    home,
    // altitude_m / speed_mps / drones intentionally absent: the KML doesn't
    // carry them, and our coverage algorithm isn't wired in yet.
  };
}

function centroid(ring: [number, number][]): [number, number] {
  // Boundary-only KML has no launch point, so the polygon's centroid is the
  // best available default until an operator/algorithm supplies a real one.
  const [latSum, lonSum] = ring.reduce(([lat, lon], [pLat, pLon]) => [lat + pLat, lon + pLon], [0, 0]);
  return [latSum / ring.length, lonSum / ring.length];
}
