import { useEffect, useRef, useState } from 'react';
import * as ROSLIB from 'roslib';
import { rosConnection } from './RosConnection';
import type { DroneTelemetry, ConnectionState } from '../types/drone';

const STALE_AFTER_MS = 5000;

/* Gazebo's navsat sensor publishes at ~25 Hz per drone (measured). With four
 * drones and no limits that is ~100 React state updates per second, each one
 * re-rendering the whole Leaflet map, and -- worse -- rosbridge queues every
 * message the browser cannot keep up with, without bound. Confirmed live as
 * the cause of a system crash: a single GCS renderer process had grown to
 * 4.75 GB (31% of RAM) with the machine at 13/14 GiB used and swapping.
 *
 * throttle_rate caps the server-side send rate; queue_length: 1 makes
 * rosbridge drop stale messages instead of accumulating a backlog. A map
 * marker does not benefit from more than a few updates per second. */
const GPS_THROTTLE_MS = 200;      // 5 Hz -- smooth enough for a moving marker
const BATTERY_THROTTLE_MS = 1000; // 1 Hz -- a percentage bar needs no more

function emptyTelemetry(namespace: string): DroneTelemetry {
  return { namespace, connected: false, gps: null, battery: null, lastUpdate: null };
}

/** Subscribes to /<namespace>/sensor_measurements/{gps,battery} for one drone. */
export function useDroneTelemetry(namespace: string): DroneTelemetry {
  const [telemetry, setTelemetry] = useState<DroneTelemetry>(() => emptyTelemetry(namespace));
  const telemetryRef = useRef(telemetry);
  telemetryRef.current = telemetry;

  useEffect(() => {
    setTelemetry(emptyTelemetry(namespace));

    let gpsTopic: ROSLIB.Topic | null = null;
    let batteryTopic: ROSLIB.Topic | null = null;
    let staleTimer: ReturnType<typeof setInterval> | null = null;

    const unsubscribeConn = rosConnection.onStateChange((state: ConnectionState) => {
      if (state !== 'connected') {
        setTelemetry((t) => ({ ...t, connected: false }));
        return;
      }

      const ros = rosConnection.getRos();

      gpsTopic = new ROSLIB.Topic({
        ros,
        name: `/${namespace}/sensor_measurements/gps`,
        messageType: 'sensor_msgs/msg/NavSatFix',
        throttle_rate: GPS_THROTTLE_MS,
        queue_length: 1,
      });
      gpsTopic.subscribe((msg: any) => {
        setTelemetry((t) => ({
          ...t,
          connected: true,
          lastUpdate: Date.now(),
          gps: { lat: msg.latitude, lon: msg.longitude, alt: msg.altitude, stamp: Date.now() },
        }));
      });

      batteryTopic = new ROSLIB.Topic({
        ros,
        name: `/${namespace}/sensor_measurements/battery`,
        messageType: 'sensor_msgs/msg/BatteryState',
        throttle_rate: BATTERY_THROTTLE_MS,
        queue_length: 1,
      });
      batteryTopic.subscribe((msg: any) => {
        setTelemetry((t) => ({
          ...t,
          connected: true,
          lastUpdate: Date.now(),
          battery: { percentage: msg.percentage, voltage: msg.voltage },
        }));
      });

      // A drone that stops publishing (crash, radio loss) should read as
      // disconnected rather than silently keep showing its last known state.
      staleTimer = setInterval(() => {
        const last = telemetryRef.current.lastUpdate;
        if (last && Date.now() - last > STALE_AFTER_MS) {
          setTelemetry((t) => ({ ...t, connected: false }));
        }
      }, 1000);
    });

    return () => {
      unsubscribeConn();
      gpsTopic?.unsubscribe();
      batteryTopic?.unsubscribe();
      if (staleTimer) clearInterval(staleTimer);
    };
  }, [namespace]);

  return telemetry;
}
