import { useEffect, useRef, useState } from 'react';
import * as ROSLIB from 'roslib';
import { rosConnection } from './RosConnection';

/** std_msgs/msg/String's wire shape — see useMissionControl.ts for why this
 * is parameterized on Topic<T> rather than cast to ROSLIB.Message. */
type RosString = { data: string };

export interface SurvivorControlStatus {
  survivor_id: string;
  action: string;
  success: boolean;
  detail: string;
}

export type SurvivorList = Record<string, [number, number]>;

/**
 * Talks to mission_file_executor.py's survivor-placement facility
 * (/gcs/survivor_control/command + /gcs/survivor_control/status +
 * /gcs/survivors/list) — add/remove/clear human-dummy models in the running
 * Gazebo sim without a restart. Follows the same connect/advertise/subscribe
 * pattern as useDroneControl.ts. `survivors` (from /gcs/survivors/list) is
 * the reconciled source of truth for rendering, not local optimistic state —
 * a page reload or a second GCS client sees the same dummies.
 */
export function useSurvivorControl() {
  const [survivors, setSurvivors] = useState<SurvivorList>({});
  const [lastStatus, setLastStatus] = useState<SurvivorControlStatus | null>(null);
  const commandTopicRef = useRef<ROSLIB.Topic<RosString> | null>(null);

  useEffect(() => {
    let statusTopic: ROSLIB.Topic<RosString> | null = null;
    let listTopic: ROSLIB.Topic<RosString> | null = null;

    const unsubscribe = rosConnection.onStateChange((state) => {
      if (state !== 'connected') return;
      const ros = rosConnection.getRos();

      commandTopicRef.current = new ROSLIB.Topic<RosString>({
        ros,
        name: '/gcs/survivor_control/command',
        messageType: 'std_msgs/msg/String',
      });
      commandTopicRef.current.advertise();

      statusTopic = new ROSLIB.Topic<RosString>({
        ros,
        name: '/gcs/survivor_control/status',
        messageType: 'std_msgs/msg/String',
        queue_length: 1,
      });
      statusTopic.subscribe((msg) => {
        try {
          setLastStatus(JSON.parse(msg.data));
        } catch {
          // ignore malformed status payloads rather than crashing the GCS
        }
      });

      listTopic = new ROSLIB.Topic<RosString>({
        ros,
        name: '/gcs/survivors/list',
        messageType: 'std_msgs/msg/String',
        queue_length: 1,
      });
      listTopic.subscribe((msg) => {
        try {
          setSurvivors(JSON.parse(msg.data));
        } catch {
          // ignore malformed list payloads rather than crashing the GCS
        }
      });
    });

    return () => {
      unsubscribe();
      statusTopic?.unsubscribe();
      listTopic?.unsubscribe();
    };
  }, []);

  function addSurvivor(lat: number, lon: number) {
    commandTopicRef.current?.publish({ data: JSON.stringify({ action: 'add', lat, lon }) });
  }

  function removeSurvivor(survivorId: string) {
    commandTopicRef.current?.publish({ data: JSON.stringify({ action: 'remove', survivor_id: survivorId }) });
  }

  function clearSurvivors() {
    commandTopicRef.current?.publish({ data: JSON.stringify({ action: 'clear' }) });
  }

  return { survivors, lastStatus, addSurvivor, removeSurvivor, clearSurvivors };
}
