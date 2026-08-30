import type { DroneDetectionState } from '../ros/useDetections';
import './DetectionPanel.css';

interface DetectionPanelProps {
  droneNamespaces: string[];
  byDrone: Record<string, DroneDetectionState>;
  total: number;
  observations: number;
}

export function DetectionPanel({
  droneNamespaces,
  byDrone,
  total,
  observations,
}: DetectionPanelProps) {
  return (
    <div className="detection-card-container">
      <div className="obsidian-card-header">
        <span>SURVIVOR DETECTIONS</span>
        <span className="obsidian-badge badge-warning">{total} FOUND</span>
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
                  {state ? `${state.count} frames (${observations} obs)` : 'No observations yet'}
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
                  {count} {count === 1 ? 'Person' : 'Persons'}
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
