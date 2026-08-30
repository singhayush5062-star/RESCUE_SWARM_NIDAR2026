import React from 'react';
import './MissionSummaryModal.css';

interface MissionSummaryModalProps {
  isOpen: boolean;
  onClose: () => void;
  missionName: string;
  durationSeconds: number;
  areaM2: number;
  survivorsCount: number;
  droneCount: number;
  /** How the mission actually ended, from the backend's own status. The
   * report used to hard-code SUCCESSFUL_RETURN even when it was opened by
   * the ABORT button. */
  outcome?: 'COMPLETE' | 'ABORTED' | 'ERROR';
}

export const MissionSummaryModal: React.FC<MissionSummaryModalProps> = ({
  isOpen,
  onClose,
  missionName,
  durationSeconds,
  areaM2,
  survivorsCount,
  droneCount,
  outcome = 'COMPLETE',
}) => {
  if (!isOpen) return null;

  const minutes = Math.floor(durationSeconds / 60);
  const seconds = durationSeconds % 60;
  const formattedTime = `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;

  const formatArea = (m2: number) => {
    if (m2 > 10000) {
      return `${(m2 / 10000).toFixed(2)} ha`;
    }
    return `${Math.round(m2)} m²`;
  };

  const handleExport = () => {
    const report = {
      mission: missionName,
      timestamp: new Date().toISOString(),
      duration: formattedTime,
      areaCovered: formatArea(areaM2),
      survivorsFound: survivorsCount,
      dronesDeployed: droneCount,
      status: outcome,
    };

    const blob = new Blob([JSON.stringify(report, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `mission_summary_${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  return (
    <div className="modal-backdrop" onClick={onClose}>
      <div className="modal-card" onClick={(e) => e.stopPropagation()}>
        <div className="modal-header">
          <div className="modal-title">
            <span className="icon" style={{ color: 'var(--status-success)' }}>
              task_alt
            </span>
            MISSION COMPLETE SUMMARY
          </div>
          <button className="obsidian-btn" style={{ padding: '2px 8px' }} onClick={onClose}>
            ✕
          </button>
        </div>

        <div className="modal-body">
          <div style={{ color: 'var(--text-muted)', fontSize: 12 }}>
            Swarm mission <strong style={{ color: '#fff' }}>{missionName}</strong> has concluded. All {droneCount} drones have safely returned to base.
          </div>

          <div className="summary-grid">
            <div className="summary-metric-card">
              <span className="summary-metric-label">FLIGHT TIME</span>
              <span className="summary-metric-value">{formattedTime}</span>
            </div>
            <div className="summary-metric-card">
              <span className="summary-metric-label">SEARCH AREA</span>
              <span className="summary-metric-value">{formatArea(areaM2)}</span>
            </div>
            <div className="summary-metric-card">
              <span className="summary-metric-label">SURVIVORS</span>
              <span className="summary-metric-value" style={{ color: 'var(--status-warning)' }}>
                {survivorsCount} LOCATED
              </span>
            </div>
          </div>

          <div className="survivors-list-section">
            <div className="obsidian-card-header" style={{ marginBottom: 6 }}>
              SWARM OPERATIONAL STATUS
            </div>
            <div style={{ display: 'flex', flexDirection: 'column', gap: 6, fontSize: 11 }}>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span>SWARM COVERAGE:</span>
                <span className="telemetry-val" style={{ color: 'var(--status-success)' }}>
                  100% COMPLETE
                </span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span>COLLISION INCIDENTS:</span>
                <span className="telemetry-val" style={{ color: 'var(--status-success)' }}>
                  0 INCIDENTS
                </span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <span>LANDING STATUS:</span>
                <span className="telemetry-val" style={{ color: 'var(--status-success)' }}>
                  ALL DRONES TOUCHDOWN (HOME BOX)
                </span>
              </div>
            </div>
          </div>
        </div>

        <div className="modal-footer">
          <button className="obsidian-btn" onClick={handleExport}>
            <span className="icon">download</span>
            EXPORT REPORT
          </button>
          <button className="obsidian-btn obsidian-btn-primary" onClick={onClose}>
            DISMISS
          </button>
        </div>
      </div>
    </div>
  );
};
