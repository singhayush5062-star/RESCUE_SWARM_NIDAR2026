import React from 'react';
import type { ActiveTab, ExecutionMode } from '../types/gcs';
import type { ConnectionState } from '../types/drone';
import './HeaderNavbar.css';

interface HeaderNavbarProps {
  activeTab: ActiveTab;
  onTabChange: (tab: ActiveTab) => void;
  executionMode: ExecutionMode;
  onModeToggle: (mode: ExecutionMode) => void;
  rosState: ConnectionState;
  missionStatusState: string;
  /** People actually found by the detector (distinct ByteTrack ids). */
  detectedCount: number;
  /** Survivors the operator placed in the simulator -- ground truth, not a
   * detection result. Kept separate because conflating the two makes the
   * detector look like it found everything the moment they were placed. */
  placedSurvivorCount: number;
  onStartMission: () => void;
  onPauseMission?: () => void;
  onRecallMission?: () => void;
  onAbortMission?: () => void;
}

export const HeaderNavbar: React.FC<HeaderNavbarProps> = ({
  activeTab,
  onTabChange,
  executionMode,
  onModeToggle,
  rosState,
  missionStatusState,
  detectedCount,
  placedSurvivorCount,
  onStartMission,
  onRecallMission,
  onAbortMission,
}) => {
  const isMissionRunning = missionStatusState === 'running' || missionStatusState === 'taking_off';

  return (
    <header className="gcs-header-nav">
      <div className="gcs-header-brand">
        <div className="brand-icon">
          <span className="icon">radar</span>
        </div>
        <div className="brand-text">
          <span className="brand-title">NIDAR RESCUESWARM</span>
          <span className="brand-subtitle">GROUND CONTROL STATION</span>
        </div>
      </div>

      {/* Navigation Tabs */}
      <nav className="gcs-header-tabs">
        <button
          className={`tab-btn ${activeTab === 'CONTROL' ? 'active' : ''}`}
          onClick={() => onTabChange('CONTROL')}
        >
          <span className="icon">dashboard</span>
          MISSION CONTROL
        </button>
        <button
          className={`tab-btn ${activeTab === 'PLANNING' ? 'active' : ''}`}
          onClick={() => onTabChange('PLANNING')}
        >
          <span className="icon">map</span>
          PLANNING
        </button>
        <button
          className={`tab-btn ${activeTab === 'MANUAL' ? 'active' : ''}`}
          onClick={() => onTabChange('MANUAL')}
        >
          <span className="icon">sports_esports</span>
          MANUAL FLIGHT
        </button>
        <button
          className={`tab-btn ${activeTab === 'VIDEO' ? 'active' : ''}`}
          onClick={() => onTabChange('VIDEO')}
        >
          <span className="icon">videocam</span>
          VIDEO & DETECTIONS
        </button>
        <button
          className={`tab-btn ${activeTab === 'LOGS' ? 'active' : ''}`}
          onClick={() => onTabChange('LOGS')}
        >
          <span className="icon">terminal</span>
          CONSOLE LOGS
        </button>
      </nav>

      {/* Execution Mode & Quick Actions */}
      <div className="gcs-header-actions">
        {/* SIM / HARDWARE Toggle */}
        <div className="mode-switcher" title="Toggle between Simulation and Hardware Testing Modes">
          <button
            className={`mode-btn ${executionMode === 'SIMULATION' ? 'active' : ''}`}
            onClick={() => onModeToggle('SIMULATION')}
          >
            SIMULATION
          </button>
          <button
            className={`mode-btn ${executionMode === 'HARDWARE' ? 'active' : ''}`}
            onClick={() => onModeToggle('HARDWARE')}
          >
            HARDWARE
          </button>
        </div>

        {/* ROS Connection Status */}
        <div className="conn-status-pill">
          <div
            className={`status-dot ${
              executionMode === 'SIMULATION'
                ? 'sim'
                : rosState === 'connected'
                ? 'online'
                : 'offline'
            }`}
          />
          <span>
            {executionMode === 'SIMULATION'
              ? 'SIM ACTIVE'
              : rosState === 'connected'
              ? 'ROS CONNECTED'
              : 'ROS OFFLINE'}
          </span>
        </div>

        {/* Survivors: found by the detector, and placed in the sim. */}
        {detectedCount > 0 && (
          <div className="obsidian-badge badge-warning" style={{ display: 'flex', alignItems: 'center', gap: 4 }}
               title="Distinct people found by the onboard detectors">
            <span className="icon" style={{ fontSize: 14 }}>person_search</span>
            {detectedCount} DETECTED
          </div>
        )}
        {placedSurvivorCount > 0 && (
          <div className="obsidian-badge" style={{ display: 'flex', alignItems: 'center', gap: 4 }}
               title="Survivors placed in the simulator (ground truth)">
            <span className="icon" style={{ fontSize: 14 }}>pin_drop</span>
            {placedSurvivorCount} PLACED
          </div>
        )}

        {/* Swarm Quick Commands */}
        <div className="quick-action-group">
          {!isMissionRunning ? (
            <button className="obsidian-btn obsidian-btn-primary" onClick={onStartMission}>
              <span className="icon">rocket_launch</span>
              LAUNCH SWARM
            </button>
          ) : (
            <>
              {/* RECALL and ABORT have no backend command behind them yet:
                  nidar_mission_executor accepts arm / disarm / takeoff /
                  set_launch_position, and its return-to-launch runs only as
                  the last phase of a mission -- there is no way to trigger
                  it, or to interrupt a running mission, from outside.
                  Disabled rather than left clickable: a live-looking ABORT
                  that silently does nothing is worse than no ABORT, because
                  the operator stops looking for another way to stop the
                  aircraft. Re-enable by passing the handlers once
                  DroneCommand gains 'rtl' and 'abort'. */}
              <button
                className="obsidian-btn"
                onClick={onRecallMission}
                disabled={!onRecallMission}
                title={onRecallMission
                  ? 'Return to Launch Site'
                  : 'Unavailable: no return-to-launch command exists in the backend yet'}
              >
                <span className="icon">keyboard_return</span>
                RECALL
              </button>
              <button
                className="obsidian-btn obsidian-btn-danger"
                onClick={onAbortMission}
                disabled={!onAbortMission}
                title={onAbortMission
                  ? 'Emergency Abort All Drones'
                  : 'Unavailable: no abort command exists in the backend yet'}
              >
                <span className="icon">warning</span>
                ABORT
              </button>
            </>
          )}
        </div>
      </div>
    </header>
  );
};
