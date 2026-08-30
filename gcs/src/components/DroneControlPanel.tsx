import { useState } from 'react';
import type { useDroneControl } from '../ros/useDroneControl';
import type { ExecutionMode } from '../types/gcs';
import './DroneControlPanel.css';

interface DroneControlPanelProps {
  droneNamespaces: string[];
  control: ReturnType<typeof useDroneControl>;
  altitudeM: number;
  executionMode?: ExecutionMode;
}

export function DroneControlPanel({
  droneNamespaces,
  control,
  altitudeM: initialAltitude,
  executionMode = 'SIMULATION',
}: DroneControlPanelProps) {
  const [target, setTarget] = useState<string>('all');
  const [targetAltitude, setTargetAltitude] = useState<number>(initialAltitude || 25);
  const [maxSpeed, setMaxSpeed] = useState<number>(2.5);
  const [isOverrideUnlocked, setIsOverrideUnlocked] = useState<boolean>(false);

  const { statusByDrone, sendCommand } = control;
  const status = target === 'all' ? null : statusByDrone[target];

  const handleNudge = (direction: string) => {
    if (direction === 'ARM') {
      sendCommand('arm', target);
    } else if (direction === 'TAKEOFF') {
      sendCommand('takeoff', target, targetAltitude);
    } else if (direction === 'DISARM') {
      sendCommand('disarm', target);
    }
  };

  // Directional nudge, hold, motor cut and a ground-speed limit all need a
  // manual velocity-command path that does not exist: DroneCommand carries
  // arm / disarm / takeoff / set_launch_position and nothing else, and
  // nothing subscribes to a nudge topic. These controls used to call
  // handleNudge('NORTH') and fall through to a console.log, so they looked
  // live and moved nothing. Disabled until that backend exists.
  const MANUAL_NUDGE_AVAILABLE = false;
  const nudgeTitle = 'Unavailable: no manual velocity command exists in the backend yet';

  return (
    <div className="manual-flight-container">
      <div className="obsidian-card-header">
        <span>MANUAL FLIGHT & CONTROL</span>
        <span className="obsidian-badge badge-primary">{executionMode}</span>
      </div>

      {/* Target Selector */}
      <div>
        <div className="control-section-header">TARGET SELECTOR</div>
        <div className="drone-selector-row">
          <button
            className={`selector-btn ${target === 'all' ? 'active' : ''}`}
            onClick={() => setTarget('all')}
          >
            ALL
          </button>
          {droneNamespaces.map((ns) => (
            <button
              key={ns}
              className={`selector-btn ${target === ns ? 'active' : ''}`}
              onClick={() => setTarget(ns)}
            >
              {ns.toUpperCase().replace('DRONE', 'D-')}
            </button>
          ))}
        </div>
      </div>

      {/* Flight Action Buttons */}
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: 6 }}>
        <button
          className="obsidian-btn"
          style={{ justifyContent: 'center' }}
          onClick={() => handleNudge('ARM')}
        >
          <span className="icon" style={{ fontSize: 14, color: 'var(--status-success)' }}>
            power_settings_new
          </span>
          ARM
        </button>
        <button
          className="obsidian-btn obsidian-btn-primary"
          style={{ justifyContent: 'center' }}
          onClick={() => handleNudge('TAKEOFF')}
        >
          <span className="icon" style={{ fontSize: 14 }}>
            flight_takeoff
          </span>
          TAKEOFF
        </button>
        <button
          className="obsidian-btn"
          style={{ justifyContent: 'center' }}
          onClick={() => handleNudge('DISARM')}
        >
          <span className="icon" style={{ fontSize: 14, color: 'var(--status-danger)' }}>
            flight_land
          </span>
          LAND
        </button>
      </div>

      {/* Setpoint Sliders */}
      <div className="setpoint-slider-group">
        <div className="slider-label-row">
          <span>ALTITUDE SETPOINT</span>
          <span className="telemetry-val" style={{ color: 'var(--primary-bright)' }}>
            {targetAltitude} m
          </span>
        </div>
        <input
          type="range"
          min="5"
          max="50"
          step="1"
          className="custom-range-slider"
          value={targetAltitude}
          onChange={(e) => setTargetAltitude(Number(e.target.value))}
        />
      </div>

      <div className="setpoint-slider-group">
        <div className="slider-label-row">
          <span>MAX GROUND SPEED (not sent)</span>
          <span className="telemetry-val" style={{ color: 'var(--primary-bright)' }}>
            {maxSpeed} m/s
          </span>
        </div>
        <input
          type="range"
          min="0.5"
          max="8.0"
          step="0.5"
          className="custom-range-slider"
          value={maxSpeed}
          disabled
          title="Unavailable: mission speed is set in the mission panel; there is no per-drone speed command"
          onChange={(e) => setMaxSpeed(Number(e.target.value))}
        />
      </div>

      {/* Manual D-Pad Joystick Nudge */}
      <div>
        <div className="control-section-header" style={{ textAlign: 'center' }}>
          MANUAL DIRECTION NUDGE
        </div>
        <div className="dpad-container">
          <div />
          <button className="dpad-btn" onClick={() => handleNudge('NORTH')}
                  disabled={!MANUAL_NUDGE_AVAILABLE} title={nudgeTitle}>
            <span className="icon">arrow_upward</span>
          </button>
          <div />

          <button className="dpad-btn" onClick={() => handleNudge('WEST')}
                  disabled={!MANUAL_NUDGE_AVAILABLE} title={nudgeTitle}>
            <span className="icon">arrow_back</span>
          </button>
          <button className="dpad-btn" onClick={() => handleNudge('HOLD')}
                  disabled={!MANUAL_NUDGE_AVAILABLE} title={nudgeTitle}>
            <span className="icon">front_hand</span>
          </button>
          <button className="dpad-btn" onClick={() => handleNudge('EAST')}
                  disabled={!MANUAL_NUDGE_AVAILABLE} title={nudgeTitle}>
            <span className="icon">arrow_forward</span>
          </button>

          <div />
          <button className="dpad-btn" onClick={() => handleNudge('SOUTH')}
                  disabled={!MANUAL_NUDGE_AVAILABLE} title={nudgeTitle}>
            <span className="icon">arrow_downward</span>
          </button>
          <div />
        </div>
      </div>

      {/* Critical Emergency Override */}
      <div style={{ marginTop: 6, paddingTop: 10, borderTop: '1px solid var(--border-dark)' }}>
        <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 6 }}>
          <span style={{ fontSize: 10, fontWeight: 700, color: 'var(--status-danger)', fontFamily: 'var(--font-mono)' }}>
            CRITICAL OVERRIDE
          </span>
          <button
            className="obsidian-btn"
            style={{ padding: '2px 6px', fontSize: 10 }}
            onClick={() => setIsOverrideUnlocked(!isOverrideUnlocked)}
          >
            <span className="icon" style={{ fontSize: 12 }}>
              {isOverrideUnlocked ? 'lock_open' : 'lock'}
            </span>
            {isOverrideUnlocked ? 'UNLOCKED' : 'LOCKED'}
          </button>
        </div>

        {isOverrideUnlocked && (
          <button
            className="obsidian-btn obsidian-btn-danger"
            style={{ width: '100%', justifyContent: 'center' }}
            disabled
            title="Unavailable: no kill-motors command exists in the backend yet"
          >
            <span className="icon">warning</span>
            EMERGENCY CUT MOTORS
          </button>
        )}
      </div>

      {status && (
        <div
          style={{
            fontSize: 10,
            fontFamily: 'var(--font-mono)',
            padding: 6,
            backgroundColor: status.success ? 'rgba(34,197,94,0.1)' : 'rgba(239,68,68,0.1)',
            border: `1px solid ${status.success ? 'var(--status-success)' : 'var(--status-danger)'}`,
            color: status.success ? 'var(--status-success)' : 'var(--status-danger)',
          }}
        >
          {status.drone_id} · {status.action}: {status.detail}
        </div>
      )}
    </div>
  );
}
