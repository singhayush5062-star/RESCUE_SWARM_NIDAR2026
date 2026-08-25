import { useEffect, useState } from 'react';
import { rosConnection } from './ros/RosConnection';
import { useDroneTelemetry } from './ros/useDroneTelemetry';
import { useMissionControl } from './ros/useMissionControl';
import { useZoneAllocation } from './ros/useZoneAllocation';
import { useMissionPlannedPaths } from './ros/useMissionPlannedPaths';
import { useDroneControl } from './ros/useDroneControl';
import { useSurvivorControl } from './ros/useSurvivorControl';
import { MapView } from './components/MapView';
import { DroneStatusPanel } from './components/DroneStatusPanel';
import { MissionLoader } from './components/MissionLoader';
import { DroneControlPanel } from './components/DroneControlPanel';
import { MappingAreaToolbar } from './components/MappingAreaToolbar';
import {
  calculateCentroid,
  createMissionFromLaunchSiteAndBoundary,
  generateRandomLaunchSiteInKml,
  generateRandomPointsInPolygon,
} from './mission/launchSiteManager';
import type { ConnectionState } from './types/drone';
import type { MissionFile } from './types/mission';
import './App.css';

const DRONE_NAMESPACES = ['drone0', 'drone1', 'drone2', 'drone3'];

function useRosbridgeState(): ConnectionState {
  const [state, setState] = useState<ConnectionState>(rosConnection.getState());
  useEffect(() => {
    const unsubscribe = rosConnection.onStateChange(setState);
    return () => {
      unsubscribe();
    };
  }, []);
  return state;
}

function App() {
  const rosbridgeState = useRosbridgeState();
  const drone0 = useDroneTelemetry(DRONE_NAMESPACES[0]);
  const drone1 = useDroneTelemetry(DRONE_NAMESPACES[1]);
  const drone2 = useDroneTelemetry(DRONE_NAMESPACES[2]);
  const drone3 = useDroneTelemetry(DRONE_NAMESPACES[3]);
  const drones = [drone0, drone1, drone2, drone3];

  const { status, loadedMission, loadMission, startMission, updateAltitude } = useMissionControl();
  const zones = useZoneAllocation(loadedMission);
  const plannedPaths = useMissionPlannedPaths(loadedMission);
  const droneControl = useDroneControl();
  const survivorControl = useSurvivorControl();

  // Interactive Mapping & Launch Site States
  const [isDrawingBoundary, setIsDrawingBoundary] = useState(false);
  const [isSettingLaunchSite, setIsSettingLaunchSite] = useState(false);
  const [isPlacingSurvivor, setIsPlacingSurvivor] = useState(false);
  const [drawnVertices, setDrawnVertices] = useState<[number, number][]>([]);

  // When a new point is clicked while drawing boundary
  function handleAddVertex(lat: number, lon: number) {
    const nextVertices = [...drawnVertices, [lat, lon] as [number, number]];
    setDrawnVertices(nextVertices);

    if (nextVertices.length >= 3) {
      const home = loadedMission?.home || calculateCentroid(nextVertices);
      const newMission: MissionFile = createMissionFromLaunchSiteAndBoundary(
        nextVertices,
        home,
        'Custom Drawn Mapping Area',
        loadedMission?.altitude_m ?? 25,
        loadedMission?.speed_mps ?? 2.0,
      );
      loadMission(newMission);
    }
  }

  // When a point is clicked to set the Home Launch Site
  function handleSetLaunchSite(lat: number, lon: number) {
    const boundary = loadedMission?.boundary || drawnVertices;
    const newMission: MissionFile = createMissionFromLaunchSiteAndBoundary(
      boundary.length >= 3 ? boundary : [[lat - 0.001, lon - 0.001], [lat + 0.001, lon - 0.001], [lat + 0.001, lon + 0.001], [lat - 0.001, lon + 0.001]],
      [lat, lon],
      loadedMission?.mission_name || 'Mapping Mission',
      loadedMission?.altitude_m ?? 25,
      loadedMission?.speed_mps ?? 2.0,
    );
    loadMission(newMission);
    setIsSettingLaunchSite(false);
  }

  // Randomly place launch site inside active boundary
  function handleRandomizeLaunchSite() {
    const boundary = loadedMission?.boundary || drawnVertices;
    if (!boundary || boundary.length < 3) return;

    const randomHome = generateRandomLaunchSiteInKml(boundary);
    handleSetLaunchSite(randomHome[0], randomHome[1]);
  }

  function handleClearBoundary() {
    setDrawnVertices([]);
    setIsDrawingBoundary(false);
  }

  // Scatter `count` random survivors inside the active boundary
  function handleAddRandomSurvivors(count: number) {
    const boundary = loadedMission?.boundary || drawnVertices;
    if (!boundary || boundary.length < 3) return;
    const points = generateRandomPointsInPolygon(boundary, count);
    points.forEach(([lat, lon]) => survivorControl.addSurvivor(lat, lon));
  }

  return (
    <div className="gcs-layout">
      <header className="gcs-header">NIDAR RescueSwarm — Ground Control Station</header>
      <div className="gcs-body">
        <div className="gcs-sidebar">
          <MissionLoader
            loadedMission={loadedMission}
            status={status}
            onLoad={loadMission}
            onStart={startMission}
            onAltitudeChange={updateAltitude}
          />
          <DroneControlPanel
            droneNamespaces={DRONE_NAMESPACES}
            control={droneControl}
            altitudeM={loadedMission?.altitude_m ?? 25}
          />
          <DroneStatusPanel drones={drones} rosbridgeState={rosbridgeState} />
        </div>
        <div className="gcs-map">
          <MappingAreaToolbar
            isDrawingBoundary={isDrawingBoundary}
            isSettingLaunchSite={isSettingLaunchSite}
            isPlacingSurvivor={isPlacingSurvivor}
            drawnVertexCount={drawnVertices.length}
            hasBoundary={!!(loadedMission?.boundary && loadedMission.boundary.length >= 3)}
            survivorCount={Object.keys(survivorControl.survivors).length}
            onToggleDrawBoundary={() => {
              setIsDrawingBoundary(!isDrawingBoundary);
              if (isSettingLaunchSite) setIsSettingLaunchSite(false);
              if (isPlacingSurvivor) setIsPlacingSurvivor(false);
            }}
            onToggleSetLaunchSite={() => {
              setIsSettingLaunchSite(!isSettingLaunchSite);
              if (isDrawingBoundary) setIsDrawingBoundary(false);
              if (isPlacingSurvivor) setIsPlacingSurvivor(false);
            }}
            onToggleSurvivorPlacement={() => {
              setIsPlacingSurvivor(!isPlacingSurvivor);
              if (isDrawingBoundary) setIsDrawingBoundary(false);
              if (isSettingLaunchSite) setIsSettingLaunchSite(false);
            }}
            onRandomizeLaunchSite={handleRandomizeLaunchSite}
            onClearBoundary={handleClearBoundary}
            onAddRandomSurvivors={handleAddRandomSurvivors}
            onClearSurvivors={survivorControl.clearSurvivors}
          />
          <MapView
            drones={drones}
            mission={loadedMission}
            zones={zones}
            plannedPaths={plannedPaths}
            isDrawingBoundary={isDrawingBoundary}
            isSettingLaunchSite={isSettingLaunchSite}
            isPlacingSurvivor={isPlacingSurvivor}
            drawnVertices={drawnVertices}
            survivors={survivorControl.survivors}
            onAddVertex={handleAddVertex}
            onSetLaunchSite={handleSetLaunchSite}
            onAddSurvivor={survivorControl.addSurvivor}
          />
        </div>
      </div>
    </div>
  );
}

export default App;
