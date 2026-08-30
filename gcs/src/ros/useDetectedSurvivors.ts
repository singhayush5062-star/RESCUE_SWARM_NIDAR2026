import { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';
import { rosConnection } from './RosConnection';

/** One survivor the swarm believes it has found, after cross-drone dedup. */
export interface DetectedSurvivor {
  survivorId: number;
  lat: number;
  lon: number;
  alt: number;
  confidence: number;
  detectingDrone: string;
  deliveryAssigned: boolean;
  deliveryComplete: boolean;
}

/**
 * Subscribes to /gcs/survivors/aggregated (nidar_msgs/msg/SurvivorList).
 *
 * This is the *algorithm's* output -- where the geotag pipeline thinks
 * survivors are. It is deliberately kept separate from useSurvivorControl's
 * list, which is where the operator *placed* ground-truth dummies. Conflating
 * them would make it impossible to see how well detection is doing, which is
 * the entire reason both exist.
 *
 * The backend publishes a full snapshot each time (see SurvivorList.msg), so
 * this replaces rather than merges: a survivor the aggregator has dropped or
 * re-merged must disappear from the map, which accumulating by id would
 * silently prevent.
 */
export function useDetectedSurvivors(): DetectedSurvivor[] {
  const [survivors, setSurvivors] = useState<DetectedSurvivor[]>([]);

  useEffect(() => {
    let topic: ROSLIB.Topic | null = null;
    const unsubscribe = rosConnection.onStateChange((state) => {
      if (state !== 'connected') {
        return;
      }
      topic = new ROSLIB.Topic({
        ros: rosConnection.getRos(),
        name: '/gcs/survivors/aggregated',
        messageType: 'nidar_msgs/msg/SurvivorList',
        queue_length: 1,
      });
      topic.subscribe((msg: any) => {
        const list: DetectedSurvivor[] = (msg?.survivors ?? []).map((s: any) => ({
          survivorId: s.survivor_id,
          lat: s.latitude,
          lon: s.longitude,
          alt: s.altitude,
          confidence: s.confidence,
          detectingDrone: s.detecting_drone_id,
          deliveryAssigned: s.delivery_assigned,
          deliveryComplete: s.delivery_complete,
        }));
        setSurvivors(list);
      });
    });
    return () => {
      unsubscribe();
      topic?.unsubscribe();
    };
  }, []);

  return survivors;
}
