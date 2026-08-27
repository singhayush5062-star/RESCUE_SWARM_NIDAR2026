#!/usr/bin/env python3
"""Captures a labelled aerial-person dataset from the simulator.

A development tool, not part of a mission: unlike every other node here it
both commands and observes, because it has to pose the scene it records.

Why this exists: implementation-plan section 2.1 (fine-tune on aerial human
imagery) turned out to be a hard prerequisite rather than later polish.
Measured live, stock COCO YOLO weights score a nadir-viewed person at
0.03-0.12 confidence even when the figure fills the frame -- COCO contains
essentially no top-down people, so this is a viewpoint gap, not a resolution
one, and no amount of flying lower fixes it. Detection at nadir does not
work at all without domain-specific weights.

The simulator is the best available source of that training data: it knows
exactly where every actor is, so labels are computed rather than drawn, and
the imagery is the exact domain the detector will run in. The tradeoff is
deliberate and bounded -- weights trained on sim renders are for sim.
Phase 8's hardware deployment still needs the real-imagery fine-tune
(VisDrone/HERIDAL/SARD) the plan calls for; this makes Phases 2-4
verifiable now instead of blocking all of them behind a data-collection
effort.

Method: fly a lawnmower over the world config's survivor-actor field at
several altitudes, grabbing a frame every capture_period. Each frame's
labels come from projecting every actor's known world position into the
image and matching that to the actor's exact pixel silhouette.

Two simulator constraints shape this, both confirmed live rather than
assumed:
  * Gazebo <actor> entities ignore SetEntityPose -- the service returns
    success and the actor does not move, because its trajectory script
    re-asserts its pose every tick. Actors can only be placed at spawn
    time, so the scene is built once and the *camera* moves instead.
  * Teleporting the drone mid-flight is not an option either: it is in
    offboard hover and the controller simply flies it back. Hence a real
    lawnmower flight, which also makes the captured imagery match what the
    detector will actually see on a mission.
"""

import math
import random
import threading
import time
from pathlib import Path

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from nidar_detection import silhouette
from nidar_detection.projection import CameraPose, Intrinsics, project_point, to_yolo_label

#: Height at which an actor's silhouette centroid sits, used as the point to
#: project. The SDF's own root pose is hip height (z=1.0); mid-body is close
#: enough that the projected point lands inside the silhouette at every scan
#: altitude, which is all the blob matching needs.
ACTOR_CENTROID_Z = 0.9


class DatasetCaptureNode(Node):

    def __init__(self):
        super().__init__('dataset_capture')

        self.declare_parameter('drone_id', 'drone0')
        self.declare_parameter('world_name', 'empty')
        self.declare_parameter('output_dir', '/tmp/nidar_detection_dataset')
        self.declare_parameter('field_size_m', 36.0)
        self.declare_parameter('world_config',
                               '/home/ayush/NIDAR/project_gazebo/config/world_swarm.yaml')
        self.declare_parameter('altitudes', [6.0, 10.0, 14.0, 19.0, 25.0])
        self.declare_parameter('speed_mps', 2.5)
        self.declare_parameter('capture_period_sec', 0.5)
        self.declare_parameter('val_fraction', 0.15)
        self.declare_parameter('match_radius_px', 60.0)
        self.declare_parameter('min_box_px', 5.0)
        self.declare_parameter('seed', 20260827)

        p = self.get_parameter
        self.drone_id = p('drone_id').value
        self.world_name = p('world_name').value
        self.field_size = float(p('field_size_m').value)
        self.world_config = p('world_config').value
        self.altitudes = list(p('altitudes').value)
        self.speed = float(p('speed_mps').value)
        self.capture_period = float(p('capture_period_sec').value)
        self.val_fraction = float(p('val_fraction').value)
        self.match_radius_px = float(p('match_radius_px').value)
        self.min_box_px = float(p('min_box_px').value)
        self.rng = random.Random(int(p('seed').value))

        self.out = Path(p('output_dir').value)
        for split in ('train', 'val'):
            (self.out / 'images' / split).mkdir(parents=True, exist_ok=True)
            (self.out / 'labels' / split).mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.frame = None
        self.frame_stamp = None
        self.intr = None
        self.actors = {}          # name -> (x, y)
        self.images_written = 0
        self.labels_written = 0
        self.empty_frames = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        cam = f'/{self.drone_id}/sensor_measurements/gimbal/camera'
        self.create_subscription(Image, f'{cam}/image_raw',
                                 self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, f'{cam}/camera_info',
                                 self._on_camera_info, qos_profile_sensor_data)

        self.optical_frame = (f'{self.drone_id}/gimbal/_0/_1/_2/camera/'
                              f'hd_camera/camera/optical_frame')

    # -- ROS plumbing --------------------------------------------------

    def _on_image(self, msg: Image):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.frame_stamp = msg.header.stamp

    def _on_camera_info(self, msg: CameraInfo):
        if self.intr is None:
            self.intr = Intrinsics.from_k(msg.k, msg.width, msg.height)
            self.get_logger().info(
                f'intrinsics fx={self.intr.fx:.1f} cx={self.intr.cx:.1f} '
                f'{self.intr.width}x{self.intr.height}')

    def spin_for(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def camera_pose(self, stamp=None):
        """The camera optical frame in `earth`.

        Looked up at the *frame's* timestamp, not now: the drone is moving
        while capturing, and labelling a frame against a pose sampled even
        100 ms later shifts every box by a quarter of a metre on the ground.
        Falls back to the latest available transform if the buffer has
        already dropped that instant.
        """
        for query in ([stamp] if stamp is not None else []) + [rclpy.time.Time()]:
            try:
                tf = self.tf_buffer.lookup_transform('earth', self.optical_frame, query)
            except Exception:  # noqa: BLE001 - fall through to the next query
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            return CameraPose((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        return None

    # -- scene ---------------------------------------------------------

    def load_actor_field(self) -> bool:
        """Read every survivor actor's ground-truth position out of the
        world config -- the same file the simulator itself was launched
        from, so the positions used for labelling are by construction the
        positions the actors are actually at.

        Deliberately NOT spawned at runtime by this tool, though the SDF and
        the spawn path both exist (nidar_survivor_manager uses them).
        Measured live: 22 actors spawned this way all reported "OK creation
        of entity" and then produced zero coloured pixels across 300
        consecutive frames of a lawnmower that covered their whole field.
        Launch-time actors from this same file render correctly, so
        labelling against runtime-spawned ones would have meant labelling
        actors that are not visibly there.
        """
        import yaml

        cfg = yaml.safe_load(Path(self.world_config).read_text())
        for obj in cfg.get('objects') or []:
            if obj.get('model_type') != 'survivor_actor':
                continue
            xyz = obj.get('xyz') or [0.0, 0.0, 0.0]
            self.actors[obj.get('model_name', f'actor_{len(self.actors)}')] = (
                float(xyz[0]), float(xyz[1]))
        self.get_logger().info(
            f'loaded {len(self.actors)} survivor actors from {self.world_config}')
        return len(self.actors) > 0

    # -- capture -------------------------------------------------------

    def capture_sample(self, index: int) -> int:
        import cv2

        frame, stamp = self.frame, self.frame_stamp
        if frame is None or self.intr is None:
            return 0
        pose = self.camera_pose(stamp)
        if pose is None:
            return 0

        boxes = silhouette.blob_boxes(silhouette.saturation_mask(frame))
        if not boxes:
            self.empty_frames += 1
            return 0

        lines, used = [], []
        for (x, y) in self.actors.values():
            projected = project_point((x, y, ACTOR_CENTROID_Z), pose, self.intr)
            if projected is None:
                continue
            if not (0 <= projected[0] < self.intr.width and 0 <= projected[1] < self.intr.height):
                continue
            candidates = [b for b in boxes if b not in used]
            match = silhouette.match_box_to_point(candidates, projected, self.match_radius_px)
            if match is None:
                continue
            used.append(match)
            box = silhouette.pad_box(match, 1.0, self.intr.width, self.intr.height)
            if box[2] < self.min_box_px or box[3] < self.min_box_px:
                continue
            lines.append(to_yolo_label(box, self.intr))

        if not lines:
            self.empty_frames += 1
            return 0

        split = 'val' if self.rng.random() < self.val_fraction else 'train'
        stem = f'f{index:06d}'
        cv2.imwrite(str(self.out / 'images' / split / f'{stem}.png'), frame)
        (self.out / 'labels' / split / f'{stem}.txt').write_text('\n'.join(lines) + '\n')
        self.images_written += 1
        self.labels_written += len(lines)
        return len(lines)

    def write_data_yaml(self):
        (self.out / 'data.yaml').write_text(
            f'path: {self.out}\ntrain: images/train\nval: images/val\n'
            'names:\n  0: person\n')


def lawnmower(half: float, spacing: float):
    """Serpentine (x, y) waypoints covering a square of half-width `half`."""
    points = []
    y = -half
    flip = False
    while y <= half + 1e-6:
        xs = [half, -half] if flip else [-half, half]
        points.extend([(x, y) for x in xs])
        flip = not flip
        y += spacing
    return points


def main():
    rclpy.init()
    node = DatasetCaptureNode()

    from as2_python_api.drone_interface_gps import DroneInterfaceGPS
    from nidar_mission_manager import geo_utils, path_planner

    node.get_logger().info('waiting for camera_info + TF ...')
    node.spin_for(6.0)
    if node.intr is None:
        node.get_logger().fatal('no camera_info -- is the sim running?')
        return 1
    if not node.load_actor_field():
        node.get_logger().fatal('no survivor actors in the world config')
        return 1
    node.spin_for(2.0)

    drone = DroneInterfaceGPS(node.drone_id, use_sim_time=True, verbose=False)
    drone.arm()
    drone.offboard()
    time.sleep(1.0)
    drone.takeoff(height=float(node.altitudes[0]), speed=1.5)
    time.sleep(2.0)
    lat0, lon0, h0 = drone.gps.origin

    index = 0
    try:
        for altitude in node.altitudes:
            # Space the legs by the camera's own swath at this altitude, so
            # the capture flight sees the field the way a real coverage
            # mission would -- same model path_planner uses for missions.
            spacing = path_planner.swath_width_m(altitude) * 0.9
            legs = lawnmower(node.field_size / 2.0, spacing)
            waypoints = []
            for (x, y) in legs:
                lat, lon = geo_utils.enu_to_latlon([(x, y)], lat0, lon0, h0)[0]
                waypoints.append([lat, lon, float(altitude)])

            node.get_logger().info(
                f'--- {altitude}m: {len(waypoints)} waypoints, leg spacing {spacing:.1f}m ---')

            done = threading.Event()

            def fly():
                try:
                    drone.follow_path(waypoints, speed=node.speed)
                except Exception as e:  # noqa: BLE001 - a bare thread's exception is otherwise lost
                    node.get_logger().error(f'follow_path failed: {e}')
                finally:
                    done.set()

            threading.Thread(target=fly, daemon=True).start()

            next_capture = time.time()
            while not done.is_set():
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.time()
                if now >= next_capture:
                    index += 1
                    node.capture_sample(index)
                    next_capture = now + node.capture_period
                    if index % 100 == 0:
                        node.get_logger().info(
                            f'  {index} frames -> {node.images_written} images, '
                            f'{node.labels_written} labels, {node.empty_frames} empty')
    finally:
        node.write_data_yaml()
        node.get_logger().info(
            f'dataset complete: {node.images_written} images, {node.labels_written} labels, '
            f'{node.empty_frames} frames with nothing in view -> {node.out}')
        try:
            drone.land(speed=0.5)
            drone.shutdown()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
