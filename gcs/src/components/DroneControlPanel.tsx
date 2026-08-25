import { useState } from 'react';
import type { useDroneControl } from '../ros/useDroneControl';
import './DroneControlPanel.css';

interface DroneControlPanelProps {
  droneNamespaces: string[];
  control: ReturnType<typeof useDroneControl>;
  altitudeM: number;
}

/**
 * Dev/test panel: manual arm / disarm / takeoff per drone (or all), outside
 * of the autonomous Start/Abort/Recall mission flow. NOT part of the
 * competition-facing GCS — the competition rules (see
 * DOCUMENTS/NIDAR_Implementation_Plan.md Phase 9.3) limit that surface to
 * exactly 3 buttons to avoid manual-intervention penalties. Gate this off
 * (or remove it) before the Phase 7 competition-ready build.
 */
export function DroneControlPanel({ droneNamespaces, control, altitudeM }: DroneControlPanelProps) {
  const [target, setTarget] = useState<string>('all');
  const { statusByDrone, sendCommand } = control;
  const status = target === 'all' ? null : statusByDrone[target];

  return (
    <div className="drone-control-panel">
      <div className="drone-control-panel__label">Manual / test controls</div>
      <select
        className="drone-control-panel__select"
        value={target}
        onChange={(e) => setTarget(e.target.value)}
      >
        <option value="all">All drones</option>
        {droneNamespaces.map((ns) => (
          <option key={ns} value={ns}>{ns}</option>
        ))}
      </select>
      <div className="drone-control-panel__buttons">
        <button
          className="drone-control-panel__button drone-control-panel__button--arm"
          onClick={() => sendCommand('arm', target)}
        >
          Arm
        </button>
        <button
          className="drone-control-panel__button drone-control-panel__button--disarm"
          onClick={() => sendCommand('disarm', target)}
        >
          Disarm
        </button>
        <button
          className="drone-control-panel__button drone-control-panel__button--takeoff"
          onClick={() => sendCommand('takeoff', target, altitudeM)}
        >
          Takeoff
        </button>
      </div>
      {status && (
        <div className={`drone-control-panel__status ${status.success ? 'drone-control-panel__status--ok' : 'drone-control-panel__status--fail'}`}>
          {status.drone_id} · {status.action}: {status.detail}
        </div>
      )}
    </div>
  );
}
