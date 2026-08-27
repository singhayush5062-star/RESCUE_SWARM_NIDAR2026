import { useEffect, useState } from 'react';
import { rosConnection } from './ros/RosConnection';
import { useDroneTelemetry } from './ros/useDroneTelemetry';
import { useMissionControl } from './ros/useMissionControl';
import { useZoneAllocation } from './ros/useZoneAllocation';
import { useMissionPlannedPaths } from './ros/useMissionPlannedPaths';
import { useMissionProgress } from './ros/useMissionProgress';
import { useDetections } from './ros/useDetections';
import { useDroneControl } from './ros/useDroneControl';
import { useSurvivorControl } from './ros/useSurvivorControl';
import { MapView } from './components/MapView';
import { DroneStatusPanel } from './components/DroneStatusPanel';
import { MissionLoader } from './components/MissionLoader';
import { MissionTimer } from './components/MissionTimer';
import { DetectionPanel } from './components/DetectionPanel';
import { VideoPanel } from './components/VideoPanel';
import { DroneControlPanel } from './components/DroneControlPanel';
import { MappingAreaToolbar } from './components/MappingAreaToolbar';
import {
  calculateCentroid,
  calculatePolygonAreaM2,
  createMissionFromLaunchSiteAndBoundary,
  defaultDroneLaunchPositions,
  generateRandomDroneLaunchPositions,
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

  const { status, loadedMission, loadMission, startMission, updateMission } = useMissionControl();
  const missionProgress = useMissionProgress();
  // Every drone's annotated feed, always subscribed. Opt-in meant a normal
  // mission showed no video at all unless the operator went and enabled it.
  const detections = useDetections(DRONE_NAMESPACES, DRONE_NAMESPACES);
  const zones = useZoneAllocation(loadedMission);
  const plannedPaths = useMissionPlannedPaths(loadedMission);
  const droneControl = useDroneControl();
  const survivorControl = useSurvivorControl();

  // Interactive Mapping & Launch Site States
  const [isDrawingBoundary, setIsDrawingBoundary] = useState(false);
  const [isSettingLaunchSite, setIsSettingLaunchSite] = useState(false);
  const [isPlacingSurvivor, setIsPlacingSurvivor] = useState(false);
  const [placingDroneNs, setPlacingDroneNs] = useState<string | null>(null);
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
    // Moving the launch site must move the drones with it, immediately.
    // createMissionFromLaunchSiteAndBoundary builds a fresh mission and so
    // drops any drone_launch_positions that were set for the OLD box -- and
    // those coordinates would in any case sit outside the new one. Placing
    // the default formation around the new centre keeps the drones inside
    // the 12ft box and gives visible confirmation the launch site took.
    const formation = defaultDroneLaunchPositions([lat, lon], DRONE_NAMESPACES);
    updateMission({ drone_launch_positions: formation });
    Object.entries(formation).forEach(([ns, [dLat, dLon]]) =>
      droneControl.setLaunchPosition(ns, dLat, dLon));
  }

  // Randomly place launch site inside active boundary
  function handleRandomizeLaunchSite() {
    const boundary = loadedMission?.boundary || drawnVertices;
    if (!boundary || boundary.length < 3) return;

    const randomHome = generateRandomLaunchSiteInKml(boundary);
    handleSetLaunchSite(randomHome[0], randomHome[1]);
  }

  // Click-to-place a single drone inside the 12ft launch box
  function handleTogglePlaceDrone(ns: string) {
    setPlacingDroneNs((prev) => (prev === ns ? null : ns));
    setIsDrawingBoundary(false);
    setIsSettingLaunchSite(false);
    setIsPlacingSurvivor(false);
  }

  function handlePlaceDrone(ns: string, lat: number, lon: number) {
    updateMission({
      drone_launch_positions: {
        ...(loadedMission?.drone_launch_positions ?? {}),
        [ns]: [lat, lon] as [number, number],
      },
    });
    // Move it in the simulator now, not just in the mission file -- otherwise
    // nothing visibly happens until Start is pressed.
    droneControl.setLaunchPosition(ns, lat, lon);
    setPlacingDroneNs(null);
  }

  function handleResetDronePositions() {
    updateMission({ drone_launch_positions: undefined });
    setPlacingDroneNs(null);
  }

  // Randomly place all 4 drones inside the fixed 12ft x 12ft launch box
  // centered on the current launch site, with safe separation.
  function handleRandomizeDronePositions() {
    const center = loadedMission?.home;
    if (!center) return;
    const positions = generateRandomDroneLaunchPositions(center, DRONE_NAMESPACES);
    updateMission({ drone_launch_positions: positions });
    Object.entries(positions).forEach(([ns, [lat, lon]]) =>
      droneControl.setLaunchPosition(ns, lat, lon));
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
          <MissionTimer progress={missionProgress} />
          <MissionLoader
            loadedMission={loadedMission}
            status={status}
            droneNamespaces={DRONE_NAMESPACES}
            onLoad={loadMission}
            onStart={startMission}
            onMissionChange={updateMission}
          />
          <DroneControlPanel
            droneNamespaces={DRONE_NAMESPACES}
            control={droneControl}
            altitudeM={loadedMission?.altitude_m ?? 25}
          />
          <DetectionPanel
            droneNamespaces={DRONE_NAMESPACES}
            byDrone={detections.byDrone}
            total={detections.total}
            observations={detections.observations}
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
            boundaryAreaM2={loadedMission?.boundary ? calculatePolygonAreaM2(loadedMission.boundary) : 0}
            altitudeM={loadedMission?.altitude_m ?? 25}
            droneCount={DRONE_NAMESPACES.length}
            droneNamespaces={DRONE_NAMESPACES}
            placingDroneNs={placingDroneNs}
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
            onRandomizeDronePositions={handleRandomizeDronePositions}
            onTogglePlaceDrone={handleTogglePlaceDrone}
            onResetDronePositions={handleResetDronePositions}
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
            placingDroneNs={placingDroneNs}
            onPlaceDrone={handlePlaceDrone}
          />
        </div>
        <VideoPanel
          droneNamespaces={DRONE_NAMESPACES}
          frames={detections.frames}
          byDrone={detections.byDrone}
          total={detections.total}
          observations={detections.observations}
        />
      </div>
    </div>
  );
}

export default App;
