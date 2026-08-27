import { useRef, useState } from 'react';
import { loadMissionFileFromInput } from '../mission/parseMissionFile';
import type { MissionFile, MissionStatus } from '../types/mission';
import './MissionLoader.css';

const DEFAULT_SCAN_ALTITUDE_M = 25;
const DEFAULT_SPEED_MPS = 2.0;

interface MissionLoaderProps {
  loadedMission: MissionFile | null;
  status: MissionStatus | null;
  droneNamespaces: string[];
  onLoad: (mission: MissionFile) => void;
  onStart: () => void;
  onMissionChange: (patch: Partial<MissionFile>) => void;
}

export function MissionLoader({
  loadedMission, status, droneNamespaces, onLoad, onStart, onMissionChange,
}: MissionLoaderProps) {
  const [error, setError] = useState<string | null>(null);
  const [showPerDrone, setShowPerDrone] = useState(false);
  const fileInputRef = useRef<HTMLInputElement>(null);

  async function handleFileChange(e: React.ChangeEvent<HTMLInputElement>) {
    const file = e.target.files?.[0];
    if (!file) return;
    try {
      const mission = await loadMissionFileFromInput(file);
      setError(null);
      onLoad(mission);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load mission file');
    } finally {
      e.target.value = '';
    }
  }

  const droneCount = loadedMission?.drones ? Object.keys(loadedMission.drones).length : 0;
  const hasWaypoints = droneCount > 0;
  const hasBoundary = !!loadedMission?.boundary && loadedMission.boundary.length >= 3;
  const canStart = loadedMission !== null && (hasWaypoints || hasBoundary) && (status === null || status.state === 'loaded');
  const isRunning = status !== null && !['idle', 'loaded', 'complete', 'error'].includes(status.state);
  const showFlightParams = loadedMission && (hasWaypoints || hasBoundary);

  function setDroneAltitude(ns: string, value: number) {
    onMissionChange({
      drone_altitudes: { ...(loadedMission?.drone_altitudes ?? {}), [ns]: value },
    });
  }

  return (
    <div className="mission-loader">
      <button className="mission-loader__button" onClick={() => fileInputRef.current?.click()} disabled={isRunning}>
        Load Mission File
      </button>
      <input ref={fileInputRef} type="file" accept=".json,.kml,application/json" hidden onChange={handleFileChange} />

      {error && <div className="mission-loader__error">{error}</div>}

      {loadedMission && (
        <div className="mission-loader__summary">
          <strong>{loadedMission.mission_name}</strong>
          <div>
            {hasWaypoints
              ? `${droneCount} drone(s) · alt ${loadedMission.altitude_m}m · ${loadedMission.speed_mps ?? DEFAULT_SPEED_MPS}m/s`
              : 'Boundary only — no flight paths yet'}
          </div>
        </div>
      )}

      {loadedMission && !hasWaypoints && hasBoundary && (
        <div className="mission-loader__pending">
          Arena boundary loaded from {loadedMission.source.toUpperCase()}. Press Start to auto-generate
          per-drone coverage zones and lawnmower flight paths.
        </div>
      )}

      {showFlightParams && (
        <>
          <label className="mission-loader__altitude">
            Flight altitude (m)
            <input
              type="number"
              min={1}
              step={1}
              value={loadedMission.altitude_m ?? DEFAULT_SCAN_ALTITUDE_M}
              disabled={isRunning}
              onChange={(e) => {
                const value = Number(e.target.value);
                if (!Number.isNaN(value) && value > 0) onMissionChange({ altitude_m: value });
              }}
            />
          </label>

          <label className="mission-loader__altitude">
            Flight speed (m/s)
            <input
              type="number"
              min={0.1}
              step={0.1}
              value={loadedMission.speed_mps ?? DEFAULT_SPEED_MPS}
              disabled={isRunning}
              onChange={(e) => {
                const value = Number(e.target.value);
                if (!Number.isNaN(value) && value > 0) onMissionChange({ speed_mps: value });
              }}
            />
          </label>

          <button
            className="mission-loader__button mission-loader__button--per-drone-toggle"
            onClick={() => setShowPerDrone((v) => !v)}
            disabled={isRunning}
          >
            {showPerDrone ? 'Hide' : 'Adjust'} per-drone altitude
          </button>

          {showPerDrone && (
            <div className="mission-loader__per-drone">
              {droneNamespaces.map((ns) => (
                <label key={ns} className="mission-loader__altitude">
                  {ns}
                  <input
                    type="number"
                    min={1}
                    step={1}
                    value={loadedMission.drone_altitudes?.[ns] ?? loadedMission.altitude_m ?? DEFAULT_SCAN_ALTITUDE_M}
                    disabled={isRunning}
                    onChange={(e) => {
                      const value = Number(e.target.value);
                      if (!Number.isNaN(value) && value > 0) setDroneAltitude(ns, value);
                    }}
                  />
                </label>
              ))}
            </div>
          )}
        </>
      )}

      <button className="mission-loader__button mission-loader__button--start" onClick={onStart} disabled={!canStart}>
        Start Mission
      </button>

      {status && (
        <div className={`mission-loader__status mission-loader__status--${status.state}`}>
          <span className="mission-loader__status-state">{status.state}</span>
          <span>{status.detail}</span>
        </div>
      )}
    </div>
  );
}
