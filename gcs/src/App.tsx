import { useEffect, useRef, useState } from 'react';
import { rosConnection } from './ros/RosConnection';
import { useDroneTelemetry } from './ros/useDroneTelemetry';
import { useMissionControl } from './ros/useMissionControl';
import { useZoneAllocation } from './ros/useZoneAllocation';
import { useMissionPlannedPaths } from './ros/useMissionPlannedPaths';
import { useMissionProgress } from './ros/useMissionProgress';
import { useDetections } from './ros/useDetections';
import { useDroneControl } from './ros/useDroneControl';
import { useSurvivorControl } from './ros/useSurvivorControl';
import { useDetectedSurvivors } from './ros/useDetectedSurvivors';
import { useMissionLog } from './ros/useMissionLog';
import { MapView } from './components/MapView';
import { DroneStatusPanel } from './components/DroneStatusPanel';
import { MissionLoader } from './components/MissionLoader';
import { MissionTimer } from './components/MissionTimer';
import { DetectionPanel } from './components/DetectionPanel';
import { VideoPanel } from './components/VideoPanel';
import { DroneControlPanel } from './components/DroneControlPanel';
import { MappingAreaToolbar } from './components/MappingAreaToolbar';
import { HeaderNavbar } from './components/HeaderNavbar';
import { LogsConsolePanel } from './components/LogsConsolePanel';
import { MissionSummaryModal } from './components/MissionSummaryModal';

import {
  DEFAULT_MAP_CENTER,
  calculateCentroid,
  calculatePolygonAreaM2,
  createMissionFromLaunchSiteAndBoundary,
  defaultDroneLaunchPositions,
  generateRandomDroneLaunchPositions,
  generateRandomPositionsNear,
  generateRandomLaunchSiteInKml,
  generateRandomPointsInPolygon,
  generateRandomBoundaryArea,
} from './mission/launchSiteManager';

import type { ConnectionState, DroneTelemetry } from './types/drone';
import type { MissionFile } from './types/mission';
import type { ActiveTab, ExecutionMode, LogEntry } from './types/gcs';

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

  // Execution Mode: SIMULATION or HARDWARE
  const [executionMode, setExecutionMode] = useState<ExecutionMode>('SIMULATION');
  const [activeTab, setActiveTab] = useState<ActiveTab>('CONTROL');

  // ROS Hooks
  const drone0 = useDroneTelemetry(DRONE_NAMESPACES[0]);
  const drone1 = useDroneTelemetry(DRONE_NAMESPACES[1]);
  const drone2 = useDroneTelemetry(DRONE_NAMESPACES[2]);
  const drone3 = useDroneTelemetry(DRONE_NAMESPACES[3]);
  const realDrones = [drone0, drone1, drone2, drone3];

  const { status, loadedMission, loadMission, startMission, updateMission } = useMissionControl();
  const missionProgress = useMissionProgress();
  const detections = useDetections(DRONE_NAMESPACES, DRONE_NAMESPACES);
  const zones = useZoneAllocation(loadedMission);
  const plannedPaths = useMissionPlannedPaths(loadedMission);
  const droneControl = useDroneControl();
  const survivorControl = useSurvivorControl();
  // Phase 3: what the geotag pipeline found, kept separate from the
  // operator-placed ground-truth dummies above.
  const detectedSurvivors = useDetectedSurvivors();
  // Backend log stream (/gcs/log <- /rosout, filtered by nidar_gcs_bridge).
  const missionLog = useMissionLog();
  // Every drone-command reply the backend has sent, echoed into the console.
  //
  // DroneControlPanel renders these, but it only exists on the MANUAL tab --
  // so a placement issued from the PLANNING toolbar failed with its
  // explanation rendered nowhere at all. Measured case: four
  // `set_launch_position: failed after 3 retries -- service never became
  // available` replies arriving while the operator watched four drones not
  // move, with no indication anything had gone wrong.
  const seenControlStatus = useRef<Set<string>>(new Set());

  // Interactive Mapping & Launch Site States
  const [isDrawingBoundary, setIsDrawingBoundary] = useState(false);
  const [isSettingLaunchSite, setIsSettingLaunchSite] = useState(false);
  const [isPlacingSurvivor, setIsPlacingSurvivor] = useState(false);
  const [placingDroneNs, setPlacingDroneNs] = useState<string | null>(null);
  const [drawnVertices, setDrawnVertices] = useState<[number, number][]>([]);

  // Logs & Summary Modal State
  // Actions taken in THIS browser. The backend's own output arrives
  // separately via useMissionLog and the two are merged for display -- the
  // console previously showed only these, seeded with two hard-coded lines
  // that described a startup that had not been verified to happen.
  const [logs, setLogs] = useState<LogEntry[]>([]);

  const [isSummaryModalOpen, setIsSummaryModalOpen] = useState(false);
  const [missionDuration, setMissionDuration] = useState(0);
  const [missionOutcome, setMissionOutcome] = useState<'COMPLETE' | 'ABORTED' | 'ERROR'>('COMPLETE');

  // Standalone Simulation Telemetry Generator
  const [simDrones, setSimDrones] = useState<Record<string, { lat: number; lon: number; alt: number; heading: number; batt: number; speed: number }>>({
    drone0: { lat: DEFAULT_MAP_CENTER[0] + 0.000000, lon: DEFAULT_MAP_CENTER[1] + 0.000000, alt: 0.0, heading: 0, batt: 98, speed: 0.0 },
    drone1: { lat: DEFAULT_MAP_CENTER[0] + 0.000008, lon: DEFAULT_MAP_CENTER[1] + 0.000006, alt: 0.0, heading: 90, batt: 97, speed: 0.0 },
    drone2: { lat: DEFAULT_MAP_CENTER[0] - 0.000012, lon: DEFAULT_MAP_CENTER[1] - 0.000014, alt: 0.0, heading: 180, batt: 99, speed: 0.0 },
    drone3: { lat: DEFAULT_MAP_CENTER[0] - 0.000022, lon: DEFAULT_MAP_CENTER[1] + 0.000016, alt: 0.0, heading: 270, batt: 96, speed: 0.0 },
  });

  // Where the swarm actually is, according to the swarm.
  //
  // Everything that needs a reference point -- the random arena centre, the
  // nearby drone scatter -- used to fall back to a coordinate literal copied
  // into four files. That is only ever right for one simulator world, and it
  // is never right for real hardware flown anywhere else. The drones publish
  // their own position; that is the authoritative answer.
  //
  // Latched on the FIRST fix rather than recomputed: it is a mission
  // reference point, and a centroid that follows four drones around the
  // arena would silently move the "random arena" and "scatter" targets to
  // wherever the swarm happened to be mid-flight.
  const [swarmOrigin, setSwarmOrigin] = useState<[number, number] | null>(null);
  useEffect(() => {
    if (swarmOrigin) return;
    const fixes = realDrones
      .filter((d) => d.gps)
      .map((d) => [d.gps!.lat, d.gps!.lon] as [number, number]);
    if (fixes.length === 0) return;
    setSwarmOrigin(calculateCentroid(fixes));
  }, [realDrones, swarmOrigin]);

  /** Mission home if one is loaded, else live GPS, else the fallback constant. */
  const mapOrigin: [number, number] =
    (loadedMission?.home as [number, number] | undefined) ?? swarmOrigin ?? DEFAULT_MAP_CENTER;

  const currentStatusState = status?.state || 'idle';

  // Echo drone-command replies into the console, failures loudest.
  useEffect(() => {
    for (const [ns, st] of Object.entries(droneControl.statusByDrone)) {
      const key = `${ns}:${st.action}:${st.success}:${st.detail}`;
      if (seenControlStatus.current.has(key)) continue;
      seenControlStatus.current.add(key);
      addLog(st.success ? 'CMD' : 'ERROR', ns,
             `${st.action}: ${st.success ? 'ok' : `FAILED — ${st.detail}`}`);
    }
    // addLog is recreated each render but only ever appends; depending on it
    // would re-run this on every log line and re-echo nothing (the seen-set
    // guards that), so it is deliberately omitted.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [droneControl.statusByDrone]);

  // Surface the report when the mission actually ends. Previously the summary
  // could only be opened by clicking ABORT, so a mission that ran to
  // completion -- the normal case -- produced no report at all.
  useEffect(() => {
    if (currentStatusState === 'complete') {
      setMissionOutcome('COMPLETE');
      setIsSummaryModalOpen(true);
    } else if (currentStatusState === 'error') {
      setMissionOutcome('ERROR');
      setIsSummaryModalOpen(true);
    }
  }, [currentStatusState]);

  // Ticker for Simulation mode
  useEffect(() => {
    const timer = setInterval(() => {
      // Only animate the mock when there is genuinely no backend. Running it
      // alongside live telemetry burned CPU producing numbers nothing showed.
      if (rosbridgeState !== 'connected') {
        const isRunning = currentStatusState === 'running' || currentStatusState === 'taking_off';

        setSimDrones((prev) => {
          const next = { ...prev };
          Object.keys(next).forEach((ns, idx) => {
            const current = next[ns];
            if (isRunning) {
              const speed = 2.2;
              const angle = (Date.now() / 1000 + idx * 1.5) % (2 * Math.PI);
              const dLat = Math.cos(angle) * 0.00002;
              const dLon = Math.sin(angle) * 0.00002;

              next[ns] = {
                ...current,
                lat: current.lat + dLat,
                lon: current.lon + dLon,
                alt: Math.min(25.0, current.alt + 0.5),
                heading: (angle * 180 / Math.PI + 360) % 360,
                batt: Math.max(10, current.batt - 0.01),
                speed: speed,
              };
            } else {
              next[ns] = {
                ...current,
                speed: 0.0,
                alt: Math.max(0.0, current.alt - 0.5),
              };
            }
          });
          return next;
        });

        if (isRunning) {
          setMissionDuration((d) => d + 1);
        }
      }
    }, 1000);

    return () => clearInterval(timer);
  }, [executionMode, rosbridgeState, currentStatusState]);

  // Real telemetry wins in BOTH modes, whenever it is actually arriving.
  //
  // This used to require executionMode === 'HARDWARE', which meant the map
  // and status panel showed the synthetic ticker below even while Gazebo was
  // publishing genuine GPS at 20 Hz -- verified live: 4/4 drones on
  // /droneN/sensor_measurements/gps, none of it reaching the screen. The
  // mode switch is about which *vehicles* are flying (simulated vs real
  // hardware); it is not a reason to discard telemetry. The simulator IS the
  // ROS backend, so its fixes are real telemetry.
  //
  // The synthetic drones remain only as an offline demo, for showing the
  // layout with no rosbridge at all -- and they now report connected:false,
  // so nothing renders them as a healthy drone.
  const effectiveDrones: DroneTelemetry[] = DRONE_NAMESPACES.map((ns, idx) => {
    const real = realDrones[idx];
    // `real.gps`, not `real.connected`. A drone whose telemetry has gone
    // stale is still a real drone at a real last-known position, and that
    // position is the single most useful thing to show an operator who has
    // just lost it. Keying on `connected` sent it back to the mock's fixed
    // start coordinate instead -- a drone that had flown 55 m away would
    // appear to teleport home the moment its feed hiccuped. MapView already
    // renders `connected: false` with the disconnected marker.
    if (real.gps) {
      return real;
    }
    const sim = simDrones[ns] || {
      lat: DEFAULT_MAP_CENTER[0], lon: DEFAULT_MAP_CENTER[1],
      alt: 0, heading: 0, batt: 100, speed: 0,
    };
    return {
      namespace: ns,
      connected: false,
      gps: { lat: sim.lat, lon: sim.lon, alt: sim.alt, stamp: Date.now() },
      battery: { percentage: sim.batt, voltage: 16.2 },
      speed: sim.speed,
      verticalSpeed: null,
      lastUpdate: Date.now(),
    };
  });

  // Backend lines plus this browser's own actions, oldest first, so a CMD the
  // operator issued sits next to the backend's response to it. Ordered by the
  // numeric sortKey -- see LogEntry, the displayed timestamp is unsortable.
  const consoleLines: LogEntry[] = [...missionLog.entries, ...logs].sort(
    (a, b) => a.sortKey - b.sortKey);

  const addLog = (level: 'INFO' | 'WARN' | 'ERROR' | 'CMD', source: string, message: string) => {
    setLogs((prev) => [
      ...prev,
      {
        id: Math.random().toString(36).substring(2, 9),
        timestamp: new Date().toLocaleTimeString(),
        sortKey: Date.now(),
        level,
        source,
        message,
      },
    ]);
  };

  // Interactive Boundary Handlers
  function handleAddVertex(lat: number, lon: number) {
    const nextVertices = [...drawnVertices, [lat, lon] as [number, number]];
    setDrawnVertices(nextVertices);

    if (nextVertices.length >= 3) {
      const home = loadedMission?.home || calculateCentroid(nextVertices);
      const newMission: MissionFile = createMissionFromLaunchSiteAndBoundary(
        nextVertices,
        home,
        'Custom Mapping Area',
        loadedMission?.altitude_m ?? 25,
        loadedMission?.speed_mps ?? 2.0,
      );
      loadMission(newMission);
      addLog('INFO', 'GCS', `Drawn boundary updated with ${nextVertices.length} vertices.`);
    }
  }

  function handleGenerateRandomBoundary() {
    const center = mapOrigin;
    const boundary = generateRandomBoundaryArea(center, 70);
    setDrawnVertices(boundary);
    const newMission: MissionFile = createMissionFromLaunchSiteAndBoundary(
      boundary,
      center,
      'Random Search Arena',
      loadedMission?.altitude_m ?? 25,
      loadedMission?.speed_mps ?? 2.0,
    );
    loadMission(newMission);
    addLog('INFO', 'GCS', 'Generated random arena mapping boundary polygon.');
  }

  // Home Launch Site Handler (12ft x 12ft Launch/Landing Station)
  function handleSetLaunchSite(lat: number, lon: number) {
    const boundary = loadedMission?.boundary || drawnVertices;
    const newMission: MissionFile = createMissionFromLaunchSiteAndBoundary(
      boundary.length >= 3 ? boundary : [[lat - 0.001, lon - 0.001], [lat + 0.001, lon - 0.001], [lat + 0.001, lon + 0.001], [lat - 0.001, lon + 0.001]],
      [lat, lon],
      loadedMission?.mission_name || 'Swarm Mission',
      loadedMission?.altitude_m ?? 25,
      loadedMission?.speed_mps ?? 2.0,
    );
    loadMission(newMission);
    setIsSettingLaunchSite(false);

    const formation = defaultDroneLaunchPositions([lat, lon], DRONE_NAMESPACES);
    updateMission({ drone_launch_positions: formation });
    Object.entries(formation).forEach(([ns, [dLat, dLon]]) =>
      droneControl.setLaunchPosition(ns, dLat, dLon));

    addLog('CMD', 'GCS', `Launch & Landing Station relocated to Lat: ${lat.toFixed(6)}, Lon: ${lon.toFixed(6)}.`);
  }

  function handleRandomizeLaunchSite() {
    const boundary = loadedMission?.boundary || drawnVertices;
    if (!boundary || boundary.length < 3) return;

    const randomHome = generateRandomLaunchSiteInKml(boundary);
    handleSetLaunchSite(randomHome[0], randomHome[1]);
  }

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
    droneControl.setLaunchPosition(ns, lat, lon);
    setPlacingDroneNs(null);
    addLog('CMD', 'GCS', `Placed ${ns} inside 12ft launch box at Lat: ${lat.toFixed(6)}, Lon: ${lon.toFixed(6)}.`);
  }

  function handleResetDronePositions() {
    updateMission({ drone_launch_positions: undefined });
    setPlacingDroneNs(null);
  }

  function handleRandomizeDronePositions() {
    const center = loadedMission?.home;
    // The button is disabled without a launch site, but the guard stays --
    // and now says why, instead of returning silently the way it used to.
    if (!center) {
      addLog('WARN', 'GCS', 'No launch site set — use SCATTER NEAR GPS, or set a launch site first.');
      return;
    }
    const positions = generateRandomDroneLaunchPositions(center, DRONE_NAMESPACES);
    updateMission({ drone_launch_positions: positions });
    Object.entries(positions).forEach(([ns, [lat, lon]]) =>
      droneControl.setLaunchPosition(ns, lat, lon));
    addLog('CMD', 'GCS', `Scattered ${Object.keys(positions).length} drones inside the 12ft launch box.`);
  }

  /** Scatter the swarm at random points near wherever its own GPS says it is.
   *
   * This is the pre-launch-site case: no 12ft box exists yet, so the backend
   * skips its box check (mission_executor `_launch_box_center is None`) and
   * accepts the placements. Once a launch site IS drawn, handleSetLaunchSite
   * moves every drone into the box and the backend enforces it from then on. */
  function handleScatterDronesNearby() {
    const center = swarmOrigin ?? (loadedMission?.home as [number, number] | undefined);
    if (!center) {
      addLog('WARN', 'GCS', 'No GPS fix from any drone yet — cannot place the swarm.');
      return;
    }
    const positions = generateRandomPositionsNear(center, DRONE_NAMESPACES);
    updateMission({ drone_launch_positions: positions });
    Object.entries(positions).forEach(([ns, [lat, lon]]) =>
      droneControl.setLaunchPosition(ns, lat, lon));
    addLog('CMD', 'GCS',
      `Scattered ${Object.keys(positions).length} drones near GPS ` +
      `${center[0].toFixed(6)}, ${center[1].toFixed(6)}.`);
  }

  function handleClearBoundary() {
    setDrawnVertices([]);
    setIsDrawingBoundary(false);
  }

  function handleAddRandomSurvivors(count: number) {
    const boundary = loadedMission?.boundary || drawnVertices;
    if (!boundary || boundary.length < 3) return;
    const points = generateRandomPointsInPolygon(boundary, count);
    points.forEach(([lat, lon]) => survivorControl.addSurvivor(lat, lon));
    addLog('INFO', 'SwarmManager', `Placed ${count} simulated survivors in search grid.`);
  }

  const handleStartMissionClick = () => {
    startMission();
    addLog('CMD', 'GCS', `Swarm mission launch sequence initiated in ${executionMode} mode.`);
  };

  // RECALL / ABORT handlers deliberately absent. They previously wrote a log
  // line claiming "RTL signal broadcast" and "Emergency landing commanded"
  // while publishing nothing at all -- a log entry asserting a command that
  // was never sent is worse than silence. HeaderNavbar disables both buttons
  // while no handler is supplied; restore them here once nidar_mission_executor
  // accepts 'rtl' and 'abort' on /nidar/drone_command.

  return (
    <div className="gcs-layout">
      {/* Obsidian Header Navigation Bar */}
      <HeaderNavbar
        activeTab={activeTab}
        onTabChange={setActiveTab}
        executionMode={executionMode}
        onModeToggle={(mode) => {
          setExecutionMode(mode);
          addLog('INFO', 'GCS', `Execution mode switched to ${mode}.`);
        }}
        rosState={rosbridgeState}
        missionStatusState={currentStatusState}
        detectedCount={detections.total}
        placedSurvivorCount={Object.keys(survivorControl.survivors).length}
        onStartMission={handleStartMissionClick}
      />

      <div className="gcs-body">
        {/* TAB 1: MISSION CONTROL */}
        {activeTab === 'CONTROL' && (
          <>
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
              <DetectionPanel
                droneNamespaces={DRONE_NAMESPACES}
                byDrone={detections.byDrone}
                total={detections.total}
                observations={detections.observations}
              />
              <DroneStatusPanel
                drones={effectiveDrones}
                rosbridgeState={rosbridgeState}
                executionMode={executionMode}
              />
            </div>
            <div className="gcs-main-content">
              <div className="gcs-map-container">
                <MapView
                  drones={effectiveDrones}
                  mission={loadedMission}
                  zones={zones}
                  plannedPaths={plannedPaths}
                  isDrawingBoundary={isDrawingBoundary}
                  isSettingLaunchSite={isSettingLaunchSite}
                  isPlacingSurvivor={isPlacingSurvivor}
                  drawnVertices={drawnVertices}
                  survivors={survivorControl.survivors}
                  detectedSurvivors={detectedSurvivors}
                  onAddVertex={handleAddVertex}
                  onSetLaunchSite={handleSetLaunchSite}
                  onAddSurvivor={survivorControl.addSurvivor}
                  placingDroneNs={placingDroneNs}
                  onPlaceDrone={handlePlaceDrone}
                />
              </div>
            </div>
          </>
        )}

        {/* TAB 2: MISSION PLANNING */}
        {activeTab === 'PLANNING' && (
          <>
            <div className="gcs-sidebar">
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
                onGenerateRandomBoundary={handleGenerateRandomBoundary}
                onRandomizeLaunchSite={handleRandomizeLaunchSite}
                onRandomizeDronePositions={handleRandomizeDronePositions}
                onScatterDronesNearby={handleScatterDronesNearby}
                hasLaunchSite={Boolean(loadedMission?.home)}
                onTogglePlaceDrone={handleTogglePlaceDrone}
                onResetDronePositions={handleResetDronePositions}
                onClearBoundary={handleClearBoundary}
                onAddRandomSurvivors={handleAddRandomSurvivors}
                onClearSurvivors={survivorControl.clearSurvivors}
              />
              <MissionLoader
                loadedMission={loadedMission}
                status={status}
                droneNamespaces={DRONE_NAMESPACES}
                onLoad={loadMission}
                onStart={startMission}
                onMissionChange={updateMission}
              />
            </div>
            <div className="gcs-main-content">
              <div className="gcs-map-container">
                <MapView
                  drones={effectiveDrones}
                  mission={loadedMission}
                  zones={zones}
                  plannedPaths={plannedPaths}
                  isDrawingBoundary={isDrawingBoundary}
                  isSettingLaunchSite={isSettingLaunchSite}
                  isPlacingSurvivor={isPlacingSurvivor}
                  drawnVertices={drawnVertices}
                  survivors={survivorControl.survivors}
                  detectedSurvivors={detectedSurvivors}
                  onAddVertex={handleAddVertex}
                  onSetLaunchSite={handleSetLaunchSite}
                  onAddSurvivor={survivorControl.addSurvivor}
                  placingDroneNs={placingDroneNs}
                  onPlaceDrone={handlePlaceDrone}
                />
              </div>
            </div>
          </>
        )}

        {/* TAB 3: MANUAL FLIGHT OPS */}
        {activeTab === 'MANUAL' && (
          <>
            <div className="gcs-sidebar">
              <DroneControlPanel
                droneNamespaces={DRONE_NAMESPACES}
                control={droneControl}
                altitudeM={loadedMission?.altitude_m ?? 25}
                executionMode={executionMode}
              />
            </div>
            <div className="gcs-main-content">
              <div className="gcs-map-container">
                <MapView
                  drones={effectiveDrones}
                  mission={loadedMission}
                  zones={zones}
                  plannedPaths={plannedPaths}
                  isDrawingBoundary={isDrawingBoundary}
                  isSettingLaunchSite={isSettingLaunchSite}
                  isPlacingSurvivor={isPlacingSurvivor}
                  drawnVertices={drawnVertices}
                  survivors={survivorControl.survivors}
                  detectedSurvivors={detectedSurvivors}
                  onAddVertex={handleAddVertex}
                  onSetLaunchSite={handleSetLaunchSite}
                  onAddSurvivor={survivorControl.addSurvivor}
                  placingDroneNs={placingDroneNs}
                  onPlaceDrone={handlePlaceDrone}
                />
              </div>
            </div>
          </>
        )}

        {/* TAB 4: VIDEO FEEDS & DETECTIONS */}
        {activeTab === 'VIDEO' && (
          <div className="gcs-main-content">
            <VideoPanel
              droneNamespaces={DRONE_NAMESPACES}
              frames={detections.frames}
              byDrone={detections.byDrone}
              total={detections.total}
              observations={detections.observations}
              executionMode={executionMode}
              drones={effectiveDrones}
            />
          </div>
        )}

        {/* TAB 5: CONSOLE LOGS & DIAGNOSTICS */}
        {activeTab === 'LOGS' && (
          <div className="gcs-main-content">
            <LogsConsolePanel
              logs={consoleLines}
              onClearLogs={() => {
                setLogs([]);
                missionLog.clear();
              }}
              droneNamespaces={DRONE_NAMESPACES}
            />
          </div>
        )}
      </div>

      {/* Mission Complete Summary Overlay */}
      <MissionSummaryModal
        isOpen={isSummaryModalOpen}
        onClose={() => setIsSummaryModalOpen(false)}
        missionName={loadedMission?.mission_name || 'Swarm Search Mission'}
        // The backend's own monotonic mission clock, not a browser interval.
        // The local counter this used to read kept ticking through pauses and
        // never reset, so the "mission duration" in an exported report was
        // whatever the tab had been open for.
        durationSeconds={missionProgress?.elapsed_time_sec ?? missionDuration}
        areaM2={loadedMission?.boundary ? calculatePolygonAreaM2(loadedMission.boundary) : 0}
        // People actually found (distinct ByteTrack ids), not survivors the
        // operator placed in the sim. The `|| 2` fallback that used to be here
        // meant an empty mission still reported two rescues.
        survivorsCount={detections.total}
        droneCount={DRONE_NAMESPACES.length}
        outcome={missionOutcome}
      />
    </div>
  );
}

export default App;
