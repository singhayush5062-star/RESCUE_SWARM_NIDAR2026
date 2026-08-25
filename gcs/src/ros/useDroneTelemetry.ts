import { useEffect, useRef, useState } from 'react';
import * as ROSLIB from 'roslib';
import { rosConnection } from './RosConnection';
import type { DroneTelemetry, ConnectionState } from '../types/drone';

const STALE_AFTER_MS = 5000;

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
