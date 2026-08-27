import { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';
import { rosConnection } from './RosConnection';

/** nidar_msgs/msg/MissionStatus's wire shape over rosbridge. Published once
 * per second by nidar_mission_clock and relayed to /gcs/mission/progress by
 * nidar_gcs_bridge. `phase` is the message's own uint8 enum. */
export interface MissionProgress {
  phase: number;
  elapsed_time_sec: number;
  mission_running: boolean;
  active_drones: number;
  total_survivors_detected: number;
  total_deliveries_complete: number;
}

/** MissionStatus.msg's phase constants, mirrored. Kept in sync by hand —
 * rosbridge sends the numeric value, not the constant name. */
export const MISSION_PHASE = {
  SETUP: 0,
  SCANNING: 1,
  DELIVERING: 2,
  RTL: 3,
  COMPLETE: 4,
  ABORTED: 5,
} as const;

export const MISSION_PHASE_LABELS: Record<number, string> = {
  [MISSION_PHASE.SETUP]: 'Setup',
  [MISSION_PHASE.SCANNING]: 'Scanning',
  [MISSION_PHASE.DELIVERING]: 'Delivering',
  [MISSION_PHASE.RTL]: 'Returning',
  [MISSION_PHASE.COMPLETE]: 'Complete',
  [MISSION_PHASE.ABORTED]: 'Aborted',
};

/**
 * Subscribes to the backend's authoritative mission clock.
 *
 * Deliberately displays the backend's own elapsed value rather than running
 * a local `setInterval` anchored to a start time: the clock is measured
 * server-side with a monotonic clock, so a browser tab that gets
 * backgrounded, throttled, or reconnected can't drift away from the number
 * the mission actually took. 1 Hz is enough for a whole-seconds readout,
 * and this project has already been driven out of memory once by
 * unthrottled rosbridge topics — see useDroneTelemetry.
 */
export function useMissionProgress() {
  const [progress, setProgress] = useState<MissionProgress | null>(null);

  useEffect(() => {
    let topic: ROSLIB.Topic<MissionProgress> | null = null;

    const unsubscribe = rosConnection.onStateChange((state) => {
      if (state !== 'connected') return;
      topic = new ROSLIB.Topic<MissionProgress>({
        ros: rosConnection.getRos(),
        name: '/gcs/mission/progress',
        messageType: 'nidar_msgs/msg/MissionStatus',
        queue_length: 1,
      });
      topic.subscribe((msg) => setProgress(msg));
    });

    return () => {
      unsubscribe();
      topic?.unsubscribe();
    };
  }, []);

  return progress;
}

/**
 * Formats mission elapsed seconds as a fixed-width clock: `M:SS` under an
 * hour, `H:MM:SS` past it. Fixed-width so the display doesn't shift
 * horizontally as digits tick over.
 */
export function formatElapsed(seconds: number): string {
  const total = Math.max(0, Math.floor(seconds));
  const hours = Math.floor(total / 3600);
  const minutes = Math.floor((total % 3600) / 60);
  const secs = total % 60;
  const pad = (n: number) => String(n).padStart(2, '0');
  return hours > 0 ? `${hours}:${pad(minutes)}:${pad(secs)}` : `${minutes}:${pad(secs)}`;
}
