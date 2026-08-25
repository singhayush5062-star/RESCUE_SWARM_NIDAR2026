import { useRef, useState } from 'react';
import { loadMissionFileFromInput } from '../mission/parseMissionFile';
import type { MissionFile, MissionStatus } from '../types/mission';
import './MissionLoader.css';

const DEFAULT_SCAN_ALTITUDE_M = 25;

interface MissionLoaderProps {
  loadedMission: MissionFile | null;
  status: MissionStatus | null;
  onLoad: (mission: MissionFile) => void;
  onStart: () => void;
  onAltitudeChange: (altitude_m: number) => void;
}

export function MissionLoader({ loadedMission, status, onLoad, onStart, onAltitudeChange }: MissionLoaderProps) {
  const [error, setError] = useState<string | null>(null);
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
              ? `${droneCount} drone(s) · alt ${loadedMission.altitude_m}m`
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

      {loadedMission && (hasWaypoints || hasBoundary) && (
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
              if (!Number.isNaN(value) && value > 0) onAltitudeChange(value);
            }}
          />
        </label>
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
