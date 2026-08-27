import { MapContainer, TileLayer, Marker, Popup, Polygon, Polyline, CircleMarker, useMap, useMapEvents } from 'react-leaflet';
import { useEffect, useRef, useState } from 'react';
import L from 'leaflet';
import type { DroneTelemetry } from '../types/drone';
import type { MissionFile } from '../types/mission';
import type { SurvivorList } from '../ros/useSurvivorControl';
import { generateDroneWaypointsAtLaunchSite, isPointInPolygon, launchBoxCorners } from '../mission/launchSiteManager';
import 'leaflet/dist/leaflet.css';
import './MapView.css';

// Leaflet marker asset setup
import markerIcon2x from 'leaflet/dist/images/marker-icon-2x.png';
import markerIcon from 'leaflet/dist/images/marker-icon.png';
import markerShadow from 'leaflet/dist/images/marker-shadow.png';

const connectedIcon = new L.Icon({
  iconUrl: markerIcon,
  iconRetinaUrl: markerIcon2x,
  shadowUrl: markerShadow,
  iconSize: [25, 41],
  iconAnchor: [12, 41],
});

const disconnectedIcon = new L.Icon({
  iconUrl: markerIcon,
  iconRetinaUrl: markerIcon2x,
  shadowUrl: markerShadow,
  iconSize: [25, 41],
  iconAnchor: [12, 41],
  className: 'marker-disconnected',
});

const PLANNED_PATH_COLORS = ['#60a5fa', '#f472b6', '#fbbf24', '#a78bfa'];

interface MapViewProps {
  drones: DroneTelemetry[];
  mission: MissionFile | null;
  zones: Record<string, [number, number][]>;
  plannedPaths: Record<string, [number, number][]>;
  isDrawingBoundary?: boolean;
  isSettingLaunchSite?: boolean;
  isPlacingSurvivor?: boolean;
  /** Namespace of the drone currently being positioned by click, or null. */
  placingDroneNs?: string | null;
  drawnVertices?: [number, number][];
  survivors?: SurvivorList;
  onAddVertex?: (lat: number, lon: number) => void;
  onSetLaunchSite?: (lat: number, lon: number) => void;
  onAddSurvivor?: (lat: number, lon: number) => void;
  onPlaceDrone?: (ns: string, lat: number, lon: number) => void;
  onDroneOutOfBox?: () => void;
}

const DEFAULT_CENTER: [number, number] = [28.682412, 77.499734];

function FitToBounds({ drones, mission }: { drones: DroneTelemetry[]; mission: MissionFile | null }) {
  const map = useMap();
  const hasDoneInitialFit = useRef(false);

  useEffect(() => {
    if (!mission || !mission.boundary || mission.boundary.length === 0) return;
    const points: [number, number][] = [...mission.boundary];
    drones.forEach((d) => d.gps && points.push([d.gps.lat, d.gps.lon]));
    map.fitBounds(L.latLngBounds(points), { padding: [80, 80], maxZoom: 19 });
    hasDoneInitialFit.current = true;
  }, [mission, map]);

  useEffect(() => {
    if (hasDoneInitialFit.current) return;
    const points: [number, number][] = [];
    drones.forEach((d) => d.gps && points.push([d.gps.lat, d.gps.lon]));
    if (points.length === 0) return;
    map.fitBounds(L.latLngBounds(points), { padding: [80, 80], maxZoom: 19 });
    hasDoneInitialFit.current = true;
  }, [drones, map]);

  return null;
}

function MapClickHandler({
  isDrawingBoundary,
  isSettingLaunchSite,
  isPlacingSurvivor,
  placingDroneNs,
  launchBox,
  boundary,
  onAddVertex,
  onSetLaunchSite,
  onAddSurvivor,
  onPlaceDrone,
  onDroneOutOfBox,
  onOutOfBounds,
}: {
  isDrawingBoundary?: boolean;
  isSettingLaunchSite?: boolean;
  isPlacingSurvivor?: boolean;
  placingDroneNs?: string | null;
  launchBox?: [number, number][] | null;
  boundary?: [number, number][];
  onAddVertex?: (lat: number, lon: number) => void;
  onSetLaunchSite?: (lat: number, lon: number) => void;
  onAddSurvivor?: (lat: number, lon: number) => void;
  onPlaceDrone?: (ns: string, lat: number, lon: number) => void;
  onDroneOutOfBox?: () => void;
  onOutOfBounds?: () => void;
}) {
  useMapEvents({
    click(e) {
      if (placingDroneNs && onPlaceDrone) {
        // Gate on the 12ft launch box here as well as in the backend, so
        // the operator gets immediate feedback instead of a mission that
        // only fails once they press Start.
        if (launchBox && !isPointInPolygon([e.latlng.lat, e.latlng.lng], launchBox)) {
          onDroneOutOfBox?.();
        } else {
          onPlaceDrone(placingDroneNs, e.latlng.lat, e.latlng.lng);
        }
      } else if (isDrawingBoundary && onAddVertex) {
        onAddVertex(e.latlng.lat, e.latlng.lng);
      } else if (isSettingLaunchSite && onSetLaunchSite) {
        onSetLaunchSite(e.latlng.lat, e.latlng.lng);
      } else if (isPlacingSurvivor && onAddSurvivor) {
        if (boundary && boundary.length >= 3 && !isPointInPolygon([e.latlng.lat, e.latlng.lng], boundary)) {
          onOutOfBounds?.();
        } else {
          onAddSurvivor(e.latlng.lat, e.latlng.lng);
        }
      }
    },
  });

  return null;
}

export function MapView({
  drones,
  mission,
  zones,
  plannedPaths,
  isDrawingBoundary = false,
  isSettingLaunchSite = false,
  isPlacingSurvivor = false,
  placingDroneNs = null,
  drawnVertices = [],
  survivors = {},
  onAddVertex,
  onSetLaunchSite,
  onAddSurvivor,
  onPlaceDrone,
  onDroneOutOfBox,
}: MapViewProps) {
  const withGps = drones.filter((d) => d.gps);
  const hasExplicitWaypoints = !!mission?.drones && Object.keys(mission.drones).length > 0;
  const pathsToRender = hasExplicitWaypoints ? mission!.drones! : plannedPaths;

  // Preview drone launch positions: the GCS-configured ones actually applied
  // by the backend (mission.drone_launch_positions) if set, else the default
  // formation offsets around the launch site.
  const launchSite = mission?.home || DEFAULT_CENTER;
  const dronePreviews = generateDroneWaypointsAtLaunchSite(launchSite[0], launchSite[1]);
  const configuredLaunchPositions = mission?.drone_launch_positions;

  const [outOfBoundsWarning, setOutOfBoundsWarning] = useState(false);
  const warningTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  function handleOutOfBounds() {
    setOutOfBoundsWarning(true);
    if (warningTimerRef.current) clearTimeout(warningTimerRef.current);
    warningTimerRef.current = setTimeout(() => setOutOfBoundsWarning(false), 3000);
  }

  useEffect(() => () => {
    if (warningTimerRef.current) clearTimeout(warningTimerRef.current);
  }, []);

  const isPlacementModeActive = isDrawingBoundary || isSettingLaunchSite || isPlacingSurvivor || !!placingDroneNs;
  const launchBox = mission?.home ? launchBoxCorners(mission.home) : null;

  return (
    <div className={`map-view-wrapper ${isPlacementModeActive ? 'map-view-wrapper--crosshair' : ''}`}>
      {outOfBoundsWarning && (
        <div className="map-view-warning">⚠️ Outside the mapping boundary — click inside the blue polygon</div>
      )}
      <MapContainer center={DEFAULT_CENTER} zoom={18} style={{ height: '100%', width: '100%' }}>
      <TileLayer
        attribution='Imagery &copy; Esri &mdash; Esri, Maxar, Earthstar Geographics'
        url="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
        maxNativeZoom={19}
        maxZoom={22}
      />
      <FitToBounds drones={drones} mission={mission} />
      <MapClickHandler
        isDrawingBoundary={isDrawingBoundary}
        isSettingLaunchSite={isSettingLaunchSite}
        isPlacingSurvivor={isPlacingSurvivor}
        placingDroneNs={placingDroneNs}
        launchBox={launchBox}
        onPlaceDrone={onPlaceDrone}
        onDroneOutOfBox={onDroneOutOfBox}
        boundary={mission?.boundary}
        onAddVertex={onAddVertex}
        onSetLaunchSite={onSetLaunchSite}
        onAddSurvivor={onAddSurvivor}
        onOutOfBounds={handleOutOfBounds}
      />

      {/* Interactive Drawn Boundary Vertices & Polygon */}
      {drawnVertices.length > 0 && (
        <>
          {drawnVertices.map((v, idx) => (
            <CircleMarker
              key={`vertex-${idx}`}
              center={v}
              radius={6}
              pathOptions={{ color: '#f59e0b', fillColor: '#fbbf24', fillOpacity: 0.9, weight: 2 }}
            >
              <Popup>Boundary Vertex #{idx + 1}</Popup>
            </CircleMarker>
          ))}

          {drawnVertices.length >= 2 && (
            <Polyline positions={drawnVertices} pathOptions={{ color: '#f59e0b', weight: 2, dashArray: '4 4' }} />
          )}

          {drawnVertices.length >= 3 && (
            <Polygon positions={drawnVertices} pathOptions={{ color: '#f59e0b', weight: 2, fillOpacity: 0.15 }} />
          )}
        </>
      )}

      {/* Active Mission & Boundary */}
      {mission && (
        <>
          {mission.boundary && mission.boundary.length >= 3 && (
            <Polygon positions={mission.boundary} pathOptions={{ color: '#3b82f6', weight: 2, fillOpacity: 0.08 }} />
          )}

          {/* Launching Site Marker */}
          <CircleMarker center={mission.home} radius={9} pathOptions={{ color: '#10b981', fillColor: '#34d399', fillOpacity: 0.9, weight: 3 }}>
            <Popup>
              <strong>🚀 Home Launching Site</strong>
              <br />
              Lat: {mission.home[0].toFixed(6)}
              <br />
              Lon: {mission.home[1].toFixed(6)}
            </Popup>
          </CircleMarker>

          {/* Fixed 12ft x 12ft launch/landing box every drone must launch
              from and land within (competition rule) -- the backend
              validates and enforces this before teleporting any drone,
              see mission_executor_node.py's LAUNCH_BOX_SIZE_M. */}
          {launchBox && (
            <Polygon
              positions={launchBox}
              pathOptions={{ color: '#34d399', weight: 1.5, dashArray: '4 4', fillOpacity: 0.05 }}
            />
          )}

          {/* Auto-generated per-drone coverage zones */}
          {Object.entries(zones).map(([ns, verts], i) => (
            <Polygon
              key={`zone-${ns}`}
              positions={verts}
              pathOptions={{ color: PLANNED_PATH_COLORS[i % PLANNED_PATH_COLORS.length], weight: 1, fillOpacity: 0.15 }}
            />
          ))}

          {/* Flight paths */}
          {Object.entries(pathsToRender).map(([ns, waypoints], i) => (
            <Polyline
              key={ns}
              positions={waypoints}
              pathOptions={{ color: PLANNED_PATH_COLORS[i % PLANNED_PATH_COLORS.length], weight: 2, dashArray: '6 6' }}
            />
          ))}
        </>
      )}

      {/* Preview Drone Placement Markers when no active connected telemetry.
          Prefer the GCS-configured positions actually applied by the
          backend (mission.drone_launch_positions) over the default
          formation offsets, so what's shown matches what Start will do. */}
      {withGps.length === 0 &&
        (configuredLaunchPositions
          ? Object.entries(configuredLaunchPositions).map(([ns, pos], idx) => (
              <CircleMarker
                key={`preview-${ns}`}
                center={pos}
                radius={5}
                pathOptions={{ color: PLANNED_PATH_COLORS[idx % PLANNED_PATH_COLORS.length], fillOpacity: 0.7 }}
              >
                <Popup>Launch Position: {ns}</Popup>
              </CircleMarker>
            ))
          : Object.entries(dronePreviews).map(([ns, waypoints], idx) => (
              <CircleMarker
                key={`preview-${ns}`}
                center={waypoints[0]}
                radius={5}
                pathOptions={{ color: PLANNED_PATH_COLORS[idx % PLANNED_PATH_COLORS.length], fillOpacity: 0.7 }}
              >
                <Popup>Launch Position: {ns} (default)</Popup>
              </CircleMarker>
            )))}

      {/* Live Connected Telemetry Markers */}
      {withGps.map((drone) => (
        <Marker
          key={drone.namespace}
          position={[drone.gps!.lat, drone.gps!.lon]}
          icon={drone.connected ? connectedIcon : disconnectedIcon}
        >
          <Popup>
            <strong>{drone.namespace}</strong>
            <br />
            {drone.connected ? 'Connected' : 'No recent telemetry'}
            <br />
            Alt: {drone.gps!.alt.toFixed(1)} m
            {drone.battery && (
              <>
                <br />
                Battery: {drone.battery.percentage.toFixed(0)}%
              </>
            )}
          </Popup>
        </Marker>
      ))}

      {/* Survivor Dummies (runtime-spawned, no sim restart) */}
      {Object.entries(survivors).map(([id, [lat, lon]]) => (
        <CircleMarker
          key={id}
          center={[lat, lon]}
          radius={7}
          className="survivor-marker-pop"
          pathOptions={{ color: '#ef4444', fillColor: '#f87171', fillOpacity: 0.9, weight: 2 }}
        >
          <Popup>🧍 Survivor: {id}</Popup>
        </CircleMarker>
      ))}
      </MapContainer>
    </div>
  );
}
