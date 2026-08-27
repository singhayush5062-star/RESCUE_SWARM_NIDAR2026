import {
  MISSION_PHASE,
  MISSION_PHASE_LABELS,
  formatElapsed,
  type MissionProgress,
} from '../ros/useMissionProgress';
import './MissionTimer.css';

interface MissionTimerProps {
  progress: MissionProgress | null;
}

/**
 * Total elapsed mission time, straight from nidar_mission_clock.
 *
 * Shows a live clock while a run is in progress and freezes on the final
 * total once it ends, so the number stays readable after landing (that
 * total is the Fast Completion scoring criterion — under 15 minutes — so it
 * needs to survive the mission ending, not blank out).
 */
export function MissionTimer({ progress }: MissionTimerProps) {
  if (!progress) return null;

  const { phase, elapsed_time_sec, mission_running, active_drones } = progress;
  const isTerminal = phase === MISSION_PHASE.COMPLETE || phase === MISSION_PHASE.ABORTED;

  // Nothing has run yet this session: no clock worth showing.
  if (!mission_running && !isTerminal && elapsed_time_sec === 0) return null;

  const modifier = mission_running
    ? 'running'
    : phase === MISSION_PHASE.ABORTED
      ? 'aborted'
      : 'complete';

  return (
    <div className={`mission-timer mission-timer--${modifier}`}>
      <div className="mission-timer__label">
        {mission_running ? 'Elapsed' : phase === MISSION_PHASE.ABORTED ? 'Ended after' : 'Total time'}
      </div>
      <div className="mission-timer__value">{formatElapsed(elapsed_time_sec)}</div>
      <div className="mission-timer__meta">
        <span className="mission-timer__phase">
          {MISSION_PHASE_LABELS[phase] ?? 'Unknown'}
        </span>
        {active_drones > 0 && (
          <span className="mission-timer__drones">{active_drones} drone(s)</span>
        )}
      </div>
    </div>
  );
}
