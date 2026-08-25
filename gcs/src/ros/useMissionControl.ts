import { useEffect, useRef, useState } from 'react';
import * as ROSLIB from 'roslib';
import { rosConnection } from './RosConnection';
import type { MissionFile, MissionStatus } from '../types/mission';

/** std_msgs/msg/String's wire shape. roslib's own bundled types (not the
 * separate @types/roslib package, which "bundler" module resolution no
 * longer picks up now that roslib ships its own .d.ts files) default
 * Topic<T>'s T to `unknown` and export no `Message` type — parameterize
 * Topic with this instead of casting to a type that doesn't resolve. */
type RosString = { data: string };

/**
 * Talks to mission_file_executor.py (project_gazebo/mission_file_executor.py)
 * over the same start/load-topic convention already used by the stock
 * behavior tree's WaitForEvent node.
 */
export function useMissionControl() {
  const [status, setStatus] = useState<MissionStatus | null>(null);
  const [loadedMission, setLoadedMission] = useState<MissionFile | null>(null);
  const loadTopicRef = useRef<ROSLIB.Topic<RosString> | null>(null);
  const startTopicRef = useRef<ROSLIB.Topic<RosString> | null>(null);

  useEffect(() => {
    let statusTopic: ROSLIB.Topic<RosString> | null = null;

    const unsubscribe = rosConnection.onStateChange((state) => {
      if (state !== 'connected') return;
      const ros = rosConnection.getRos();

      loadTopicRef.current = new ROSLIB.Topic<RosString>({
        ros,
        name: '/gcs/mission_load',
        messageType: 'std_msgs/msg/String',
      });
      startTopicRef.current = new ROSLIB.Topic<RosString>({
        ros,
        name: '/gcs/mission_start',
        messageType: 'std_msgs/msg/String',
      });
      // Advertise as soon as we connect, not lazily on first publish, so the
      // topic is already established by the time the operator clicks Load/Start.
      loadTopicRef.current.advertise();
      startTopicRef.current.advertise();

      statusTopic = new ROSLIB.Topic<RosString>({
        ros,
        name: '/gcs/mission_status',
        messageType: 'std_msgs/msg/String',
      });
      statusTopic.subscribe((msg) => {
        try {
          setStatus(JSON.parse(msg.data));
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

  function loadMission(mission: MissionFile) {
    setLoadedMission(mission);
    loadTopicRef.current?.publish({ data: JSON.stringify(mission) });
  }

  function startMission() {
    startTopicRef.current?.publish({ data: 'start' });
  }

  // Lets the operator adjust flight altitude before Start. _on_load just
  // re-stores whatever it last received, so re-publishing with the edited
  // value keeps the backend's copy in sync — no new topic needed.
  function updateAltitude(altitude_m: number) {
    setLoadedMission((prev) => {
      if (!prev) return prev;
      const updated = { ...prev, altitude_m };
      loadTopicRef.current?.publish({ data: JSON.stringify(updated) });
      return updated;
    });
  }

  return { status, loadedMission, loadMission, startMission, updateAltitude };
}
