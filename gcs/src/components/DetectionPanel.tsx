import type { DroneDetectionState } from '../ros/useDetections';
import './DetectionPanel.css';

interface DetectionPanelProps {
  droneNamespaces: string[];
  byDrone: Record<string, DroneDetectionState>;
  /** People, reconciled across the swarm by the geotag aggregator. This is
   *  the only number here that answers "how many survivors are there". */
  survivorsFound: number;
  /** Raw per-drone ByteTrack ids, summed. Counts one person once per drone
   *  that saw them, plus once per re-acquired track -- a detector-activity
   *  measure, never a survivor count. */
  trackCount: number;
}

export function DetectionPanel({
  droneNamespaces,
  byDrone,
  survivorsFound,
  trackCount,
}: DetectionPanelProps) {
  return (
    <div className="detection-card-container">
      <div className="obsidian-card-header">
        <span>SURVIVOR DETECTIONS</span>
        <span
          className="obsidian-badge badge-warning"
          title="Distinct people after cross-drone reconciliation (geotag aggregator)"
        >
          {survivorsFound} FOUND
        </span>
      </div>

      {/* The per-drone rows below add up to more than the badge, and that is
          correct: they are raw tracks from each camera, and the same person
          seen by three drones is three tracks but one survivor. Saying so
          outright beats letting an operator add up the column and conclude
          the badge is broken. */}
      <div style={{ fontSize: 9, color: 'var(--text-subtle)', fontFamily: 'var(--font-mono)', marginBottom: 4 }}>
        {trackCount} RAW TRACKS ACROSS {droneNamespaces.length} DRONES &middot; DUPLICATES REMOVED BY POSITION
      </div>

      <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
        {droneNamespaces.map((ns) => {
          const state = byDrone[ns];
          const count = state ? state.trackIds.size : 0;

          return (
            <div key={ns} className="detection-item-row">
              <div className="detection-info">
                <div className="detection-title" style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
                  <span
                    style={{
                      width: 6,
                      height: 6,
                      borderRadius: '50%',
                      backgroundColor: count > 0 ? 'var(--status-warning)' : 'var(--text-subtle)',
                    }}
                  />
                  {ns.toUpperCase()}
                </div>
                <span className="detection-sub">
                  {/* This drone's own observation count. It used to print the
                      swarm-wide `observations` total on every row, so all four
                      rows showed the same number regardless of drone. */}
                  {state ? `${state.count} observation${state.count === 1 ? '' : 's'}` : 'No observations yet'}
                </span>
              </div>

              <div style={{ textAlign: 'right' }}>
                <span
                  className="telemetry-val"
                  style={{
                    color: count > 0 ? 'var(--status-warning)' : 'var(--text-muted)',
                    fontSize: 12,
                  }}
                >
                  {/* "Persons" was a claim this number cannot support: a
                      track is one camera's view of someone, and one person
                      generates a new track on every pass that re-acquires
                      them. */}
                  {count} {count === 1 ? 'track' : 'tracks'}
                </span>
                <div style={{ fontSize: 9, color: 'var(--text-subtle)', fontFamily: 'var(--font-mono)' }}>
                  {state && state.bestConfidence > 0 ? `${(state.bestConfidence * 100).toFixed(0)}% CONF` : '0%'}
                </div>
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}
