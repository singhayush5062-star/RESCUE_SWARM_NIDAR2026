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

import json
import math
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, NavSatFix
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped
from nidar_msgs.msg import DetectionResult, ZoneAllocation

from nidar_detection import detector as det
from nidar_detection import survey_gate

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
#: dispatches on what it points at, so .pt / .onnx / .engine / a
#: *_ncnn_model/ directory all work here. See
#: project_gazebo/models/detection/README.md.
DEFAULT_MODEL_PATH = '/home/ayush/NIDAR/project_gazebo/models/detection/nidar_person.pt'

#: Inference input size to fall back to when a PyTorch model has to run on
#: CPU. Cost scales with the square of it, so 640 -> 416 is ~2.4x cheaper.
#: Deliberately NOT applied to the ncnn backend: measured on this box, ncnn
#: at the full 640 (122 ms) is still faster than PyTorch cut to 416 (213 ms),
#: so degrading ncnn's input would give up accuracy the model was trained
#: with and buy nothing.
CPU_FALLBACK_INPUT_SIZE = 416


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
        # When inference lands on CPU and an ncnn export sits beside the
        # weights, load that instead -- 3x faster for the same output. Set
        # false to force the literal model_path regardless of device.
        self.declare_parameter('prefer_ncnn_on_cpu', True)
        # Only infer while this drone is actually flying the survey pattern at
        # survey height -- see nidar_detection/survey_gate.py for the measured
        # reason (six phantom survivors, four of them on the launch pad).
        # Left switchable because this node is also run standalone, outside any
        # mission, for dataset capture and for checking the detector is alive;
        # with the gate on and no mission running it would correctly do nothing
        # at all, which is indistinguishable from a broken node.
        self.declare_parameter('gate_on_survey', True)
        self.declare_parameter('altitude_tolerance_m',
                               survey_gate.DEFAULT_ALTITUDE_TOLERANCE_M)
        # Also require the drone to be INSIDE its allocated zone. Without
        # this the gate opens the moment the coverage phase starts, while
        # each drone is still transiting the 8-10 m from the launch box to
        # its zone -- at scan altitude, in the running state. Measured: the
        # first survivor record appears 7 s into the phase, and three
        # phantoms landed within 5 m of the launch box centre because of it.
        self.declare_parameter('require_in_zone', True)

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

        # Work out where inference will actually run, and on which export,
        # BEFORE constructing the detector -- because the answer decides the
        # input size. Both helpers are pure functions of the path and device,
        # so asking them here and letting PersonDetector ask them again is
        # deterministic, not a second opinion that could disagree.
        prefer_ncnn = bool(self.get_parameter('prefer_ncnn_on_cpu').value)
        device = det.resolve_device(str(self.get_parameter('device').value))
        resolved_path = det.resolve_model_path(model_path, device, prefer_ncnn)
        on_ncnn = det.is_ncnn_model(resolved_path)

        input_size = int(self.get_parameter('input_size').value)
        if device == 'cpu' and not on_ncnn:
            # PyTorch on CPU only: see CPU_FALLBACK_INPUT_SIZE.
            input_size = min(input_size, CPU_FALLBACK_INPUT_SIZE)

        self._bridge = CvBridge()
        self._detector = det.PersonDetector(
            model_path=model_path,
            confidence_threshold=float(self.get_parameter('confidence_threshold').value),
            iou_threshold=float(self.get_parameter('nms_threshold').value),
            input_size=input_size,
            device=str(self.get_parameter('device').value),
            tracker=str(self.get_parameter('tracker').value),
            prefer_ncnn_on_cpu=prefer_ncnn,
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

        # --- survey gate inputs -------------------------------------------
        self._gate_on_survey = bool(self.get_parameter('gate_on_survey').value)
        self._altitude_tolerance = float(self.get_parameter('altitude_tolerance_m').value)
        self._mission_state = None
        self._scan_altitude = None
        self._altitude = None
        self._gate_open = None      # None so the first decision always logs
        self._frames_gated = 0
        self._require_in_zone = bool(self.get_parameter('require_in_zone').value)
        self._zone = []             # this drone's allocated zone, [(lat, lon), ...]
        self._fix = None            # this drone's own (lat, lon)

        if self._gate_on_survey:
            self.create_subscription(String, '/nidar/mission_status',
                                     self._on_mission_status, 10)
            # BEST_EFFORT to match the state estimator's publisher, the same
            # trap already documented for the camera subscription above.
            self.create_subscription(PoseStamped,
                                     f'/{self.drone_id}/self_localization/pose',
                                     self._on_pose, qos_profile_sensor_data)
            if self._require_in_zone:
                self.create_subscription(ZoneAllocation, '/nidar/zone_allocation',
                                         self._on_zone, 10)
                self.create_subscription(NavSatFix,
                                         f'/{self.drone_id}/sensor_measurements/gps',
                                         self._on_fix, qos_profile_sensor_data)

        # Periodic heartbeat so a run that detects nothing can be told apart
        # from a run where no frames ever arrived -- the difference between
        # "the model missed them" and "the camera topic is wrong", which is
        # otherwise invisible in a headless sim.
        self.create_timer(10.0, self._log_heartbeat)

        # Falling back to CPU is not a minor slowdown here: four YOLO models
        # on CPU drove this machine to load average 65 and starved Gazebo's
        # renderer and DDS along with it, so camera frames arrived every 4-6
        # seconds instead of every 250 ms. That presents as "the video feed
        # lags", with nothing in the detection logs obviously wrong -- so say
        # it loudly, with the fix, rather than letting it look like a
        # transport problem.
        if self._detector.device == 'cpu' and self._detector.backend != 'ncnn':
            self.get_logger().warn(
                f'{self.drone_id}: running PyTorch inference on CPU, the slowest '
                f'combination available (~380 ms/frame measured). With one detector '
                f'per drone this will saturate the machine and lag the whole '
                f'simulation. Two independent fixes, either of which is enough: '
                f'(1) if this box has an NVIDIA GPU torch cannot currently reach, '
                f'"CUDA unknown error" after a suspend is usually cleared by '
                f'sudo rmmod nvidia_uvm && sudo modprobe nvidia_uvm; '
                f'(2) export the weights to ncnn -- '
                f'yolo export model={model_path} format=ncnn -- which writes '
                f'{det.NCNN_MODEL_SUFFIX} beside them and is picked up automatically, '
                f'3x faster at full resolution.')

        self.get_logger().info(
            f'detection ready for {self.drone_id} | backend={self._detector.backend} '
            f'| device={self._detector.device} | imgsz={self._detector.input_size} '
            f'| model={self._detector.model_path} | {rate:g} Hz '
            f'| conf>={self._detector.confidence_threshold} '
            f'| tracker={self._detector.tracker or "off"} '
            f'| camera={camera_topic}')

    # ------------------------------------------------------------------

    def _on_mission_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self._mission_state = payload.get('state')
        # Only read on a status that carries it: the executor publishes the
        # height with 'running', and clobbering it with None on the next
        # 'returning' would close the gate for the wrong reason.
        if payload.get('scan_altitude_m') is not None:
            self._scan_altitude = float(payload['scan_altitude_m'])

    def _on_pose(self, msg: PoseStamped):
        self._altitude = msg.pose.position.z

    def _on_zone(self, msg: ZoneAllocation):
        # One topic carries every drone's zone; keep only this drone's.
        if msg.drone_id != self.drone_id:
            return
        self._zone = [(p.latitude, p.longitude) for p in msg.zone_vertices]

    def _on_fix(self, msg: NavSatFix):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self._fix = (msg.latitude, msg.longitude)

    def _survey_gate_open(self) -> bool:
        """Whether to infer on this frame, logging every transition.

        The transition log matters more than it looks: a silently-closed gate
        and a dead detector produce exactly the same symptom -- no detections
        and a blank feed -- and this is the line that tells them apart."""
        if not self._gate_on_survey:
            return True
        is_open = survey_gate.is_surveying(
            self._mission_state, self._altitude, self._scan_altitude,
            self._altitude_tolerance)
        if is_open and self._require_in_zone:
            is_open = bool(self._fix) and survey_gate.point_in_polygon(
                self._fix[0], self._fix[1], self._zone)
        if is_open != self._gate_open:
            self._gate_open = is_open
            if is_open:
                self.get_logger().info(
                    f'[gate] OPEN -- surveying at {self._altitude:.1f}m '
                    f'(scan {self._scan_altitude:.1f}m +/- {self._altitude_tolerance:g}m)')
            else:
                alt = 'unknown' if self._altitude is None else f'{self._altitude:.1f}m'
                in_zone = ('n/a' if not self._require_in_zone
                           else bool(self._fix) and survey_gate.point_in_polygon(
                               self._fix[0], self._fix[1], self._zone))
                self.get_logger().info(
                    f'[gate] CLOSED -- state={self._mission_state!r} alt={alt} '
                    f'in_zone={in_zone}; not inferring')
        return is_open

    def _on_image(self, msg: Image):
        self._frames_seen += 1

        if not self._survey_gate_open():
            self._frames_gated += 1
            return

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
            f'skipped by survey gate {self._frames_gated}, '
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
