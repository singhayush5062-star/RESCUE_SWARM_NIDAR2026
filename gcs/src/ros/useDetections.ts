import { useEffect, useRef, useState } from 'react';
import * as ROSLIB from 'roslib';
import { rosConnection } from './RosConnection';

/** nidar_msgs/msg/DetectionResult as it arrives over rosbridge. */
export interface DetectionResult {
  detection_id: number;
  confidence: number;
  bbox_x: number;
  bbox_y: number;
  bbox_w: number;
  bbox_h: number;
  drone_id: string;
  /** ByteTrack identity, stable across frames for one physical person as
   * seen by this drone. -1 means the tracker has not confirmed a track yet. */
  track_id: number;
}

/** DetectionResult.track_id value meaning "no identity assigned yet". */
export const UNTRACKED = -1;

export interface DroneDetectionState {
  /** Raw detection messages seen — observations, not people. A survivor in
   * view for 30 frames contributes 30. Useful only as a liveness signal. */
  count: number;
  /** Distinct ByteTrack ids, i.e. how many separate people this drone has
   * actually found. This is the number that means something. */
  trackIds: Set<number>;
  /** Highest confidence seen so far — a quick read on detector health. */
  bestConfidence: number;
  /** Wall-clock ms of the most recent detection, for staleness display. */
  lastSeen: number;
}

/** sensor_msgs/msg/CompressedImage over rosbridge: `data` is base64. */
interface CompressedImage {
  format: string;
  data: string;
}

const DETECTION_THROTTLE_MS = 200;
/** Deliberately FASTER than the detector's ~2 Hz output rate.
 *
 * throttle_rate is a minimum spacing rosbridge enforces before forwarding the
 * next message, so it adds latency on top of the source rate rather than
 * replacing it: at 500 ms a frame produced 10 ms after the last delivery
 * waited a further 490 ms before the operator saw it, on top of the camera
 * (250 ms) and inference (500 ms) intervals. Setting it below the source
 * period means a new frame goes out as soon as it exists.
 *
 * This does NOT increase bandwidth: the detector only publishes ~2 frames a
 * second, so nothing extra is created, and queue_length: 1 guarantees a
 * backlog is dropped rather than played out stale. The memory incident that
 * made this cautious involved *raw* 3.6 MB frames on an unthrottled topic;
 * these are ~7 KB JPEGs. */
const FEED_THROTTLE_MS = 100;

/**
 * Subscribes to detection results from every drone, plus each drone's
 * annotated camera feed.
 *
 * Feeds are subscribed lazily — only for `feedNamespaces` — because the
 * detection node encodes a JPEG only while something is actually subscribed
 * (it checks get_subscription_count). Handing this an empty list therefore
 * costs the backend nothing at all, which is what lets the panel default to
 * showing one feed rather than four.
 */
export function useDetections(
  droneNamespaces: string[],
  feedNamespaces: string[] = [],
  resetKey?: unknown,
) {
  const [byDrone, setByDrone] = useState<Record<string, DroneDetectionState>>({});
  const [frames, setFrames] = useState<Record<string, string>>({});
  const feedKey = feedNamespaces.join(',');
  const namespaceKey = droneNamespaces.join(',');
  const framesRef = useRef(frames);
  framesRef.current = frames;

  // Clear accumulated tracker state on a new mission, the same way
  // useZoneAllocation clears its zones. Without this the sets below only ever
  // grow: a browser tab left open across three trials still holds trial one's
  // ByteTrack ids, so every count is the sum of every run since the last
  // manual refresh. Ids are also not comparable across runs -- the detector
  // restarts numbering -- so carrying them forward is meaningless as well as
  // inflating.
  useEffect(() => {
    setByDrone({});
  }, [resetKey]);

  // Detection results — one subscription for the whole swarm, fanned in by
  // nidar_gcs_bridge.
  useEffect(() => {
    let topic: ROSLIB.Topic<DetectionResult> | null = null;
    const unsubscribe = rosConnection.onStateChange((state) => {
      if (state !== 'connected') return;
      topic = new ROSLIB.Topic<DetectionResult>({
        ros: rosConnection.getRos(),
        name: '/gcs/detections',
        messageType: 'nidar_msgs/msg/DetectionResult',
        throttle_rate: DETECTION_THROTTLE_MS,
        queue_length: 1,
      });
      topic.subscribe((msg) => {
        setByDrone((prev) => {
          const existing = prev[msg.drone_id];
          // Copy rather than mutate: React compares by reference, and adding
          // to the existing Set in place would not re-render.
          const trackIds = new Set(existing?.trackIds ?? []);
          if (msg.track_id !== undefined && msg.track_id !== UNTRACKED) {
            trackIds.add(msg.track_id);
          }
          return {
            ...prev,
            [msg.drone_id]: {
              count: (existing?.count ?? 0) + 1,
              trackIds,
              bestConfidence: Math.max(existing?.bestConfidence ?? 0, msg.confidence),
              lastSeen: Date.now(),
            },
          };
        });
      });
    });
    return () => {
      unsubscribe();
      topic?.unsubscribe();
    };
  }, [namespaceKey]);

  // Annotated camera feeds — one subscription per requested drone.
  useEffect(() => {
    const topics: ROSLIB.Topic<CompressedImage>[] = [];
    const wanted = feedKey ? feedKey.split(',') : [];

    const unsubscribe = rosConnection.onStateChange((state) => {
      if (state !== 'connected') return;
      const ros = rosConnection.getRos();
      for (const ns of wanted) {
        const topic = new ROSLIB.Topic<CompressedImage>({
          ros,
          name: `/${ns}/detection/image_annotated/compressed`,
          messageType: 'sensor_msgs/msg/CompressedImage',
          throttle_rate: FEED_THROTTLE_MS,
          queue_length: 1,
        });
        topic.subscribe((msg) => {
          setFrames((prev) => ({ ...prev, [ns]: `data:image/jpeg;base64,${msg.data}` }));
        });
        topics.push(topic);
      }
    });

    return () => {
      unsubscribe();
      topics.forEach((t) => t.unsubscribe());
      // Drop stale frames so a feed that has been switched off does not keep
      // showing its last image as though it were live.
      setFrames((prev) => {
        const next = { ...prev };
        for (const ns of wanted) delete next[ns];
        return next;
      });
    };
  }, [feedKey]);

  // TRACKS, not people -- and deliberately not named `total` any more.
  //
  // Track ids are unique only within one drone's own stream, so summing them
  // counts one survivor once per drone that saw them, plus once more every
  // time a lawnmower pass re-acquires a track it had lost. Measured: 41 for
  // an arena holding 20 people. This number was previously wired straight
  // into every "N FOUND" badge and into the exported mission report.
  //
  // It is still worth having -- it is the honest measure of detector
  // activity per drone -- but the count of PEOPLE now comes from
  // useDetectedSurvivors, which reconciles across drones geographically
  // (nidar_geotag's survivor_aggregator_node). Callers that want people must
  // use that; the name change is what stops this being picked up by mistake.
  const trackCount = Object.values(byDrone).reduce((sum, d) => sum + d.trackIds.size, 0);
  const observations = Object.values(byDrone).reduce((sum, d) => sum + d.count, 0);
  return { byDrone, frames, trackCount, observations };
}
