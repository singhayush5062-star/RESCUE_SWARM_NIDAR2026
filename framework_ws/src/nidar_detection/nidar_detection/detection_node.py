#!/usr/bin/env python3
"""Detects people in one drone's camera stream.

One node, one task, one drone. Four of these run concurrently in the swarm,
each namespaced to its own drone -- the same shape AS2 already uses for
per-drone nodes (state_estimator, controller_manager, motion behaviors), and
the reason this is not a single node subscribing to four cameras: on
hardware each of these runs on its own drone's Jetson, next to its own
camera, and never sees the others.

Implements section 2.2 of DOCUMENTS/NIDAR_Implementation_Plan.md. It does
detection only -- turning a pixel bbox into a lat/lon is Phase 3's geotag
node, which consumes the DetectionResult messages published here.
"""

import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from nidar_msgs.msg import DetectionResult

from nidar_detection import detector as det

#: Camera topic AS2's gimbal_speed + hd_camera payload pair actually
#: publishes on. NOT the flat `sensor_measurements/camera` the older
#: architecture docs assume -- that difference is recorded in the
#: implementation plan's own Phase 0.3 note, and getting it wrong means this
#: node subscribes successfully to a topic nobody publishes and silently
#: detects nothing forever.
DEFAULT_CAMERA_TOPIC = 'sensor_measurements/gimbal/camera/image_raw'

#: The single weights path the whole system loads. Swapping detection models
#: -- including dropping in retrained overhead-person weights -- is a copy
#: over this file, with no code, config, or launch change; ultralytics
#: dispatches on the extension, so .pt / .onnx / .engine all work here. See
#: project_gazebo/models/detection/README.md.
DEFAULT_MODEL_PATH = '/home/ayush/NIDAR/project_gazebo/models/detection/nidar_person.pt'


class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection')

        self.declare_parameter('drone_id', 'drone0')
        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        self.declare_parameter('camera_topic', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('device', 'auto')
        # Throttled well below the camera's own rate on purpose. Inference
        # is the expensive part and four of these share one GPU beside a
        # full 4-drone flight stack; the plan's own section 2.2 calls for
        # 2-5 FPS in sim. A drone at 2 m/s covers 1 m between frames, well
        # inside the ~9 m camera footprint at scan altitude, so nothing is
        # flown past unseen.
        self.declare_parameter('inference_rate_hz', 2.0)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('annotated_max_width', 640)
        self.declare_parameter('jpeg_quality', 70)
        # ByteTrack keeps one identity per physical person across frames, so
        # the same survivor is not counted once per frame. Empty string
        # disables tracking and reverts to plain per-frame detection.
        self.declare_parameter('tracker', 'bytetrack.yaml')

        self.drone_id = self.get_parameter('drone_id').value
        model_path = self.get_parameter('model_path').value
        if not model_path:
            self.get_logger().fatal(
                'model_path parameter is required (path to YOLO weights, e.g. yolov8n.pt)')
            raise SystemExit(1)

        rate = float(self.get_parameter('inference_rate_hz').value)
        self._min_interval = 1.0 / rate if rate > 0 else 0.0
        self._last_inference = 0.0
        self._detection_id = 0
        self._frames_seen = 0
        self._frames_processed = 0
        self._detections_total = 0
        # Distinct ByteTrack ids seen. This -- not _detections_total -- is the
        # number of people found: a survivor in view for 30 frames produces 30
        # detections but one track id.
        self._track_ids = set()
        self._publish_annotated = bool(self.get_parameter('publish_annotated').value)
        self._annotated_max_width = int(self.get_parameter('annotated_max_width').value)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self._bridge = CvBridge()
        self._detector = det.PersonDetector(
            model_path=model_path,
            confidence_threshold=float(self.get_parameter('confidence_threshold').value),
            iou_threshold=float(self.get_parameter('nms_threshold').value),
            input_size=int(self.get_parameter('input_size').value),
            device=str(self.get_parameter('device').value),
            tracker=str(self.get_parameter('tracker').value),
        )

        camera_topic = self.get_parameter('camera_topic').value or \
            f'/{self.drone_id}/{DEFAULT_CAMERA_TOPIC}'

        self.results_pub = self.create_publisher(
            DetectionResult, f'/{self.drone_id}/detection/results', 10)
        self.annotated_pub = self.create_publisher(
            Image, f'/{self.drone_id}/detection/image_annotated', 1)
        self.annotated_compressed_pub = self.create_publisher(
            CompressedImage, f'/{self.drone_id}/detection/image_annotated/compressed', 1)

        # qos_profile_sensor_data (BEST_EFFORT): a RELIABLE subscription is
        # incompatible with a BEST_EFFORT publisher and would receive
        # nothing at all, whereas a BEST_EFFORT subscription works against
        # publishers of either durability. Dropping a camera frame under
        # load is also the correct behaviour here -- the next one is 33ms
        # away and this node only wants every Nth frame anyway.
        self.create_subscription(Image, camera_topic, self._on_image, qos_profile_sensor_data)

        # Periodic heartbeat so a run that detects nothing can be told apart
        # from a run where no frames ever arrived -- the difference between
        # "the model missed them" and "the camera topic is wrong", which is
        # otherwise invisible in a headless sim.
        self.create_timer(10.0, self._log_heartbeat)

        self.get_logger().info(
            f'detection ready for {self.drone_id} | device={self._detector.device} '
            f'| model={model_path} | {rate:g} Hz | conf>={self._detector.confidence_threshold} '
            f'| tracker={self._detector.tracker or "off"} '
            f'| camera={camera_topic}')

    # ------------------------------------------------------------------

    def _on_image(self, msg: Image):
        self._frames_seen += 1

        now = time.monotonic()
        if self._min_interval and (now - self._last_inference) < self._min_interval:
            return
        self._last_inference = now

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001 - a bad frame must not kill the node
            self.get_logger().warn(f'could not convert frame: {e}')
            return

        try:
            detections = self._detector.detect(frame)
        except Exception as e:  # noqa: BLE001 - inference failure must not kill the node
            self.get_logger().error(f'inference failed: {e}')
            return

        self._frames_processed += 1
        self._detections_total += len(detections)

        for d in detections:
            self._detection_id += 1
            result = DetectionResult()
            # Carry the *camera frame's* stamp, not now(): Phase 3 pairs each
            # detection with the drone pose at capture time, and restamping
            # here would silently attribute it to wherever the drone had
            # moved to by the time inference finished.
            result.header.stamp = msg.header.stamp
            result.header.frame_id = msg.header.frame_id
            result.detection_id = self._detection_id
            result.confidence = float(d.confidence)
            result.bbox_x = float(d.x)
            result.bbox_y = float(d.y)
            result.bbox_w = float(d.w)
            result.bbox_h = float(d.h)
            result.drone_id = self.drone_id
            result.track_id = int(d.track_id)
            if d.track_id != det.UNTRACKED:
                self._track_ids.add(int(d.track_id))
            self.results_pub.publish(result)

        if detections:
            best = max(d.confidence for d in detections)
            ids = sorted({d.track_id for d in detections if d.track_id != det.UNTRACKED})
            self.get_logger().info(
                f'[detect] {len(detections)} person(s), best conf {best:.2f} '
                f'| tracks in frame {ids or "none yet"} '
                f'| unique tracks so far {len(self._track_ids)}')

        self._publish_preview(msg, frame, detections)

    def _publish_preview(self, msg: Image, frame, detections):
        """Publish the annotated frame, but only in the formats someone is
        actually subscribed to.

        Annotating and JPEG-encoding every frame for nobody is pure waste
        with four of these running; get_subscription_count() makes each
        stream cost nothing until a tool (rqt_image_view) or the GCS asks
        for it.
        """
        if not self._publish_annotated:
            return
        want_raw = self.annotated_pub.get_subscription_count() > 0
        want_compressed = self.annotated_compressed_pub.get_subscription_count() > 0
        if not (want_raw or want_compressed):
            return

        annotated = det.annotate(frame, detections)

        if want_raw:
            out = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out.header = msg.header
            self.annotated_pub.publish(out)

        if want_compressed:
            import cv2
            preview = det.scale_to_width(annotated, self._annotated_max_width)
            ok, buf = cv2.imencode(
                '.jpg', preview, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
            if not ok:
                return
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.annotated_compressed_pub.publish(out)

    def _log_heartbeat(self):
        if self._frames_seen == 0:
            self.get_logger().warn(
                '[detect] no camera frames received yet -- check the camera topic name '
                'and that the sim is publishing images')
            return
        self.get_logger().info(
            f'[detect] frames seen {self._frames_seen}, inferred {self._frames_processed}, '
            f'detections {self._detections_total}, '
            f'unique people (track ids) {len(self._track_ids)}')


def main():
    rclpy.init()
    try:
        node = DetectionNode()
    except SystemExit:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
