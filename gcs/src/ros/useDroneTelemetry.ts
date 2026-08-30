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
const TWIST_THROTTLE_MS = 250;    // 4 Hz -- a speed readout is read, not tracked

/* Above this, the sample is an artefact and is dropped rather than shown.
 *
 * self_localization/twist is differentiated from position, so any
 * DISCONTINUOUS move of the model produces a velocity that never happened.
 * Teleporting a drone to a launch position is exactly such a move, and it is
 * a routine operator action. Measured on this sim: a 25 m relocation emitted
 * 43.8 m/s across 4 consecutive samples (~60 ms), and the 35 m relocation at
 * the start of a mission emitted 896 m/s. Both would have flashed on the
 * SPEED readout as if measured.
 *
 * 25 m/s (90 km/h) is far beyond anything this survey airframe flies -- real
 * mission speeds here are 1-2 m/s -- while sitting well below the smallest
 * artefact observed, so no genuine reading is at risk. The sample is
 * discarded, not clamped: showing "25.0 m/s" would be a wrong number that
 * looks like a right one, and the previous good value is the honest thing to
 * keep on screen for the ~60 ms an artefact lasts. */
const MAX_PLAUSIBLE_SPEED_MPS = 25.0;

function emptyTelemetry(namespace: string): DroneTelemetry {
  return {
    namespace,
    connected: false,
    gps: null,
    battery: null,
    speed: null,
    verticalSpeed: null,
    lastUpdate: null,
  };
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
    let twistTopic: ROSLIB.Topic | null = null;
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

      // AS2's state estimator publishes the drone's own velocity here, in
      // its body frame (frame_id is `<ns>/base_link` -- confirmed live).
      // A rotation preserves vector length, so the magnitude of the linear
      // components is the speed regardless of which frame it is expressed
      // in; only the individual axes would need transforming, and the panel
      // shows a magnitude. Vertical speed is taken from `z` and is exact
      // while the drone is level, which is every part of a survey flight
      // where a climb rate is worth reading.
      twistTopic = new ROSLIB.Topic({
        ros,
        name: `/${namespace}/self_localization/twist`,
        messageType: 'geometry_msgs/msg/TwistStamped',
        throttle_rate: TWIST_THROTTLE_MS,
        queue_length: 1,
      });
      twistTopic.subscribe((msg: any) => {
        const v = msg?.twist?.linear;
        if (!v) return;
        const speed = Math.hypot(v.x ?? 0, v.y ?? 0, v.z ?? 0);
        if (!Number.isFinite(speed) || speed > MAX_PLAUSIBLE_SPEED_MPS) return;
        setTelemetry((t) => ({
          ...t,
          connected: true,
          lastUpdate: Date.now(),
          speed,
          verticalSpeed: v.z ?? 0,
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
      twistTopic?.unsubscribe();
      if (staleTimer) clearInterval(staleTimer);
    };
  }, [namespace]);

  return telemetry;
}
