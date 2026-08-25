import type { DroneTelemetry, ConnectionState } from '../types/drone';
import './DroneStatusPanel.css';

interface DroneStatusPanelProps {
  drones: DroneTelemetry[];
  rosbridgeState: ConnectionState;
}

export function DroneStatusPanel({ drones, rosbridgeState }: DroneStatusPanelProps) {
  return (
    <div className="status-panel">
      <div className={`rosbridge-status rosbridge-status--${rosbridgeState}`}>
        rosbridge: {rosbridgeState}
      </div>
      <ul className="drone-list">
        {drones.map((drone) => (
          <li key={drone.namespace} className={`drone-card ${drone.connected ? 'connected' : 'disconnected'}`}>
            <div className="drone-card__header">
              <span className="drone-card__name">{drone.namespace}</span>
              <span className="drone-card__dot" />
            </div>
            {drone.gps ? (
              <div className="drone-card__row">
                {drone.gps.lat.toFixed(6)}, {drone.gps.lon.toFixed(6)} · {drone.gps.alt.toFixed(1)}m
              </div>
            ) : (
              <div className="drone-card__row drone-card__row--muted">No GPS fix</div>
            )}
            {drone.battery ? (
              <div className="drone-card__row">Battery: {drone.battery.percentage.toFixed(0)}%</div>
            ) : (
              <div className="drone-card__row drone-card__row--muted">No battery data</div>
            )}
          </li>
        ))}
      </ul>
    </div>
  );
}
