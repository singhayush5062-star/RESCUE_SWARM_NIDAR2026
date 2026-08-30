import type { DroneTelemetry, ConnectionState } from '../types/drone';
import type { ExecutionMode } from '../types/gcs';
import './DroneStatusPanel.css';

interface DetailedTelemetry {
  heading?: number;
  satellites?: number;
  rssi?: number;
  ekfOk?: boolean;
  armed?: boolean;
  mode?: string;
}

interface DroneStatusPanelProps {
  drones: (DroneTelemetry & DetailedTelemetry)[];
  rosbridgeState: ConnectionState;
  executionMode: ExecutionMode;
  onSelectDrone?: (ns: string) => void;
  selectedDroneNs?: string | null;
}

export function DroneStatusPanel({
  drones,
  rosbridgeState,
  executionMode,
  onSelectDrone,
  selectedDroneNs,
}: DroneStatusPanelProps) {
  return (
    <div className="drone-status-container">
      <div className="obsidian-card-header">
        <span>SWARM TELEMETRY</span>
        <span className="obsidian-badge badge-primary">
          {`${executionMode} · ROS: ${rosbridgeState.toUpperCase()}`}
        </span>
      </div>

      <div className="drone-card-grid">
        {drones.map((drone) => {
          const isSelected = selectedDroneNs === drone.namespace;
          // `drone.connected` only, never the execution mode. Treating
          // SIMULATION as "always connected" painted all four dots green and
          // every badge ONLINE even with the whole backend down -- the panel
          // whose entire job is telling the operator which drones are alive.
          const isConnected = drone.connected;
          const alt = drone.gps?.alt ?? 0.0;
          // Same rule as sats/rssi/ekf below: null means nothing has
          // published a velocity, which must not render as a measured 0.0.
          const speed = drone.speed;
          const vSpeed = drone.verticalSpeed;
          const battPercent = drone.battery?.percentage ?? 100;
          const battVoltage = drone.battery?.voltage ?? 16.8;
          // Undefined means "this telemetry is not published yet", not a
          // reading. Showing 14 sats / -62 dBm for a field nothing publishes
          // is indistinguishable from a real measurement, so show nothing.
          const sats = drone.satellites;
          const rssi = drone.rssi;
          const ekf = drone.ekfOk;

          return (
            <div
              key={drone.namespace}
              className={`drone-status-card ${isSelected ? 'active-drone' : ''}`}
              onClick={() => onSelectDrone?.(drone.namespace)}
              style={{ cursor: onSelectDrone ? 'pointer' : 'default' }}
            >
              <div className="drone-card-top">
                <div className="drone-title-group">
                  <span
                    className="drone-card__dot"
                    style={{
                      backgroundColor: isConnected
                        ? 'var(--status-success)'
                        : 'var(--status-danger)',
                      boxShadow: isConnected ? '0 0 6px var(--status-success)' : 'none',
                    }}
                  />
                  <span className="drone-name">{drone.namespace}</span>
                </div>
                <div style={{ display: 'flex', gap: 4 }}>
                  <span
                    className={`obsidian-badge ${
                      isConnected ? 'badge-success' : 'badge-danger'
                    }`}
                  >
                    {isConnected ? (drone.mode || 'ONLINE') : 'OFFLINE'}
                  </span>
                  <span className="obsidian-badge badge-primary">
                    {executionMode === 'SIMULATION' ? 'SIM' : 'HW'}
                  </span>
                </div>
              </div>

              <div className="drone-metrics-grid">
                <div className="metric-item">
                  <span className="metric-label">ALTITUDE</span>
                  <span className="metric-value">{alt.toFixed(1)} m</span>
                </div>

                <div className="metric-item">
                  <span className="metric-label">SPEED</span>
                  <span className="metric-value">
                    {speed === null || speed === undefined
                      ? '—'
                      : `${speed.toFixed(1)} m/s`}
                    {vSpeed !== null && vSpeed !== undefined && Math.abs(vSpeed) >= 0.1
                      ? ` ${vSpeed > 0 ? '↑' : '↓'}${Math.abs(vSpeed).toFixed(1)}`
                      : ''}
                    {rssi !== undefined ? ` (${rssi} dBm)` : ''}
                  </span>
                </div>

                <div className="metric-item">
                  <span className="metric-label">BATTERY</span>
                  <span
                    className="metric-value"
                    style={{
                      color:
                        battPercent < 20
                          ? 'var(--status-danger)'
                          : battPercent < 50
                          ? 'var(--status-warning)'
                          : 'var(--text-main)',
                    }}
                  >
                    {battPercent.toFixed(0)}% ({battVoltage.toFixed(1)}V)
                  </span>
                </div>

                <div className="metric-item">
                  <span className="metric-label">GPS / EKF</span>
                  <span className="metric-value">
                    {sats !== undefined ? `${sats} SATS` : 'SATS —'} ·{' '}
                    <span style={{ color: ekf === false ? 'var(--status-danger)' : 'var(--status-success)' }}>
                      {ekf === undefined ? 'EKF —' : ekf ? 'EKF OK' : 'ERR'}
                    </span>
                  </span>
                </div>

                <div className="metric-item" style={{ gridColumn: 'span 2' }}>
                  <span className="metric-label">POSITION</span>
                  <span className="metric-value" style={{ fontSize: 10 }}>
                    {drone.gps ? `${drone.gps.lat.toFixed(6)}, ${drone.gps.lon.toFixed(6)}` : 'NO GPS FIX'}
                  </span>
                </div>
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}
