import { useState } from 'react';
import type { DroneDetectionState } from '../ros/useDetections';
import './VideoPanel.css';

interface VideoPanelProps {
  droneNamespaces: string[];
  frames: Record<string, string>;
  byDrone: Record<string, DroneDetectionState>;
  /** Unique people found (distinct ByteTrack ids) across the swarm. */
  total: number;
  /** Raw detection messages — liveness only, not a person count. */
  observations: number;
}

/** A feed with no new frame for this long is treated as stalled. */
const STALE_AFTER_MS = 6000;

/**
 * Live annotated camera feeds, one per drone, always on.
 *
 * Feeds used to be opt-in per drone, which meant a normal mission showed
 * nothing at all unless the operator knew to go turn them on. They are
 * always subscribed now: the annotated stream is JPEG at 640x480, ~7 KB a
 * frame at 2 Hz, so all four together cost ~56 KB/s — the memory incident
 * that made this cautious was *raw* 3.6 MB frames, which is a different
 * thing by three orders of magnitude.
 *
 * Clicking a feed expands it full-screen, which is the only practical way to
 * read a bounding box and its track id on a 1280x960 frame scaled into a
 * column.
 */
export function VideoPanel({
  droneNamespaces, frames, byDrone, total, observations,
}: VideoPanelProps) {
  const [expanded, setExpanded] = useState<string | null>(null);
  const now = Date.now();

  return (
    <div className="video-panel">
      <div className="video-panel__header">
        <div className="video-panel__title">Detected persons</div>
        <div className="video-panel__count">{total}</div>
        <div className="video-panel__sub">
          {observations} detection{observations === 1 ? '' : 's'} · deduplicated by ByteTrack
        </div>
      </div>

      <div className="video-panel__feeds">
        {droneNamespaces.map((ns) => {
          const frame = frames[ns];
          const state = byDrone[ns];
          const live = !!frame && (!state || now - state.lastSeen < STALE_AFTER_MS);
          return (
            <figure key={ns} className="video-panel__feed">
              <figcaption className="video-panel__caption">
                <span className={`video-panel__dot ${live ? 'video-panel__dot--live' : ''}`} />
                <span className="video-panel__name">{ns}</span>
                <span className="video-panel__tracks">
                  {state ? `${state.trackIds.size} person${state.trackIds.size === 1 ? '' : 's'}` : '0 persons'}
                </span>
              </figcaption>
              {frame ? (
                <button
                  className="video-panel__frame-btn"
                  onClick={() => setExpanded(ns)}
                  title={`Expand ${ns}`}
                >
                  <img src={frame} alt={`${ns} annotated camera feed`} />
                </button>
              ) : (
                <div className="video-panel__no-signal">
                  Waiting for feed…
                </div>
              )}
            </figure>
          );
        })}
      </div>

      {expanded && (
        <div
          className="video-panel__overlay"
          onClick={() => setExpanded(null)}
          role="dialog"
          aria-label={`${expanded} camera feed`}
        >
          <div className="video-panel__overlay-inner" onClick={(e) => e.stopPropagation()}>
            <div className="video-panel__overlay-bar">
              <strong>{expanded}</strong>
              <span>
                {byDrone[expanded]
                  ? `${byDrone[expanded].trackIds.size} person(s) tracked · best ${byDrone[expanded].bestConfidence.toFixed(2)}`
                  : 'no detections yet'}
              </span>
              <button onClick={() => setExpanded(null)} aria-label="Close">✕</button>
            </div>
            {frames[expanded] ? (
              <img src={frames[expanded]} alt={`${expanded} annotated camera feed, enlarged`} />
            ) : (
              <div className="video-panel__no-signal">Waiting for feed…</div>
            )}
          </div>
        </div>
      )}
    </div>
  );
}
