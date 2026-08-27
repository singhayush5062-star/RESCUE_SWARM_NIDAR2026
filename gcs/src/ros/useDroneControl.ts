import { useEffect, useRef, useState } from 'react';
import * as ROSLIB from 'roslib';
import { rosConnection } from './RosConnection';

/** std_msgs/msg/String's wire shape — see useMissionControl.ts for why this
 * is parameterized on Topic<T> rather than cast to ROSLIB.Message. */
type RosString = { data: string };

export interface DroneControlStatus {
  drone_id: string;
  action: string;
  success: boolean;
  detail: string;
}

/**
 * Talks to mission_file_executor.py's manual drone-control facility
 * (/gcs/drone_control/command + /gcs/drone_control/status) — dev/test-only
 * arm/disarm/takeoff outside of a full autonomous mission run. Follows the
 * same connect/advertise/subscribe pattern as useMissionControl.ts.
 */
export function useDroneControl() {
  const [statusByDrone, setStatusByDrone] = useState<Record<string, DroneControlStatus>>({});
  const commandTopicRef = useRef<ROSLIB.Topic<RosString> | null>(null);

  useEffect(() => {
    let statusTopic: ROSLIB.Topic<RosString> | null = null;

    const unsubscribe = rosConnection.onStateChange((state) => {
      if (state !== 'connected') return;
      const ros = rosConnection.getRos();

      commandTopicRef.current = new ROSLIB.Topic<RosString>({
        ros,
        name: '/gcs/drone_control/command',
        messageType: 'std_msgs/msg/String',
      });
      commandTopicRef.current.advertise();

      statusTopic = new ROSLIB.Topic<RosString>({
        ros,
        name: '/gcs/drone_control/status',
        messageType: 'std_msgs/msg/String',
        queue_length: 1,
      });
      statusTopic.subscribe((msg) => {
        try {
          const status: DroneControlStatus = JSON.parse(msg.data);
          setStatusByDrone((prev) => ({ ...prev, [status.drone_id]: status }));
        } catch {
          // ignore malformed status payloads rather than crashing the GCS
        }
      });
    });

    return () => {
      unsubscribe();
      statusTopic?.unsubscribe();
    };
  }, []);

  function sendCommand(action: 'arm' | 'disarm' | 'takeoff', droneId: string, altitudeM?: number) {
    const payload: Record<string, unknown> = { action, drone_id: droneId };
    if (altitudeM !== undefined) payload.altitude_m = altitudeM;
    commandTopicRef.current?.publish({ data: JSON.stringify(payload) });
  }

  /** Move a landed drone to (lat, lon) immediately, so the operator sees it
   * happen in Gazebo at placement time rather than only when the mission
   * starts. The backend re-applies the same positions at mission start, so
   * the interactive and mission paths stay consistent. */
  function setLaunchPosition(droneId: string, lat: number, lon: number) {
    commandTopicRef.current?.publish({
      data: JSON.stringify({ action: 'set_launch_position', drone_id: droneId, lat, lon }),
    });
  }

  return { statusByDrone, sendCommand, setLaunchPosition };
}
