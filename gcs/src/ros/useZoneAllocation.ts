import { useEffect, useState } from 'react';
import * as ROSLIB from 'roslib';
import { rosConnection } from './RosConnection';

export type ZoneAllocations = Record<string, [number, number][]>;

/**
 * Subscribes to /gcs/mission/zone_allocation (nidar_msgs/msg/ZoneAllocation).
 * One message per drone; accumulates into { droneId: [[lat,lon], ...] }.
 * `resetKey` clears accumulated state on a new mission load (pass the
 * loadedMission object — a fresh reference every load).
 */
export function useZoneAllocation(resetKey: unknown): ZoneAllocations {
  const [zones, setZones] = useState<ZoneAllocations>({});

  useEffect(() => setZones({}), [resetKey]);

  useEffect(() => {
    let topic: ROSLIB.Topic | null = null;
    const unsubscribe = rosConnection.onStateChange((state) => {
      if (state !== 'connected') return;
      topic = new ROSLIB.Topic({
        ros: rosConnection.getRos(),
        name: '/gcs/mission/zone_allocation',
        messageType: 'nidar_msgs/msg/ZoneAllocation',
      });
      topic.subscribe((msg: any) => {
        // geographic_msgs/GeoPoint entries arrive as {latitude, longitude,
        // altitude} objects over rosbridge, not [lat, lon] tuples.
        const verts: [number, number][] = msg.zone_vertices.map((p: any) => [p.latitude, p.longitude]);
        setZones((prev) => ({ ...prev, [msg.drone_id]: verts }));
      });
    });
    return () => {
      unsubscribe();
      topic?.unsubscribe();
    };
  }, []);

  return zones;
}
