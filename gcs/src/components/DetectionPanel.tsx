import { useState } from 'react';
import type { DroneDetectionState } from '../ros/useDetections';
import './DetectionPanel.css';

interface DetectionPanelProps {
  droneNamespaces: string[];
  byDrone: Record<string, DroneDetectionState>;
  /** Unique people found (distinct ByteTrack ids), not raw messages. */
  total: number;
  /** Raw detection messages — a liveness signal, not a person count. */
  observations: number;
}

/** A detection older than this is shown as stale rather than current. */
const STALE_AFTER_MS = 5000;

/**
 * Per-drone detector health: how many distinct people each drone has found,
 * how many raw detections that came from, and best confidence.
 *
 * The camera feeds themselves live in VideoPanel, which has room for them —
 * this sits in the 280px sidebar and is a numbers panel only.
 */
export function DetectionPanel({
  droneNamespaces, byDrone, total, observations,
}: DetectionPanelProps) {
  const [expanded, setExpanded] = useState(true);
  const now = Date.now();

  return (
    <div className="detection-panel">
      <button
        className="detection-panel__header"
        onClick={() => setExpanded((v) => !v)}
        aria-expanded={expanded}
      >
        <span>Detection</span>
        <span
          className="detection-panel__total"
          title={`${total} distinct people tracked, from ${observations} raw detections`}
        >
          {total} {total === 1 ? 'person' : 'people'}
        </span>
        <span className="detection-panel__chevron">{expanded ? '▾' : '▸'}</span>
      </button>

      {expanded && (
        <div className="detection-panel__body">
          {droneNamespaces.map((ns) => {
            const state = byDrone[ns];
            const stale = !state || now - state.lastSeen > STALE_AFTER_MS;
            return (
              <div key={ns} className="detection-panel__row">
                <span className={`detection-panel__dot ${stale ? '' : 'detection-panel__dot--active'}`} />
                <span className="detection-panel__name">{ns}</span>
                <span className="detection-panel__count">
                  {state ? `${state.trackIds.size} tracked` : '—'}
                </span>
                <span className="detection-panel__conf">
                  {state ? `${state.count} det · ${state.bestConfidence.toFixed(2)}` : ''}
                </span>
              </div>
            );
          })}
        </div>
      )}
    </div>
  );
}
