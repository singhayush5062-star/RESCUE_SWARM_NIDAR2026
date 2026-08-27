import { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';
import { rosConnection } from './RosConnection';

/** std_msgs/msg/String's wire shape — see useMissionControl.ts for why this
 * is parameterized on Topic<T> rather than cast to ROSLIB.Message. */
type RosString = { data: string };

export type PlannedPaths = Record<string, [number, number][]>;

/**
 * Subscribes to /gcs/mission/planned_paths (std_msgs/msg/String, JSON
 * {droneId: [[lat,lon], ...]}) — the auto-generated lawnmower waypoints
 * mission_file_executor.py publishes after splitting a boundary-only
 * mission. Shaped identically to MissionFile.drones so MapView can render
 * it with the same Polyline code used for explicit JSON-waypoint missions.
 */
export function useMissionPlannedPaths(resetKey: unknown): PlannedPaths {
  const [paths, setPaths] = useState<PlannedPaths>({});

  useEffect(() => setPaths({}), [resetKey]);

  useEffect(() => {
    let topic: ROSLIB.Topic<RosString> | null = null;
    const unsubscribe = rosConnection.onStateChange((state) => {
      if (state !== 'connected') return;
      topic = new ROSLIB.Topic<RosString>({
        ros: rosConnection.getRos(),
        name: '/gcs/mission/planned_paths',
        messageType: 'std_msgs/msg/String',
        queue_length: 1,
      });
      topic.subscribe((msg) => {
        try {
          setPaths(JSON.parse(msg.data));
        } catch {
          // ignore malformed payloads, matches useMissionControl.ts's convention
        }
      });
    });
    return () => {
      unsubscribe();
      topic?.unsubscribe();
    };
  }, []);

  return paths;
}
