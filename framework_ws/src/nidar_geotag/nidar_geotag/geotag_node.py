"""Turns one drone's pixel-space detections into world-frame survivor tags.

One node per drone, namespaced -- the same shape nidar_detection already
uses, and the same "one node, one task" split as the rest of this workspace.
This node's single job is: given a bounding box and where the camera was
when it saw it, say where on the ground that is.

    /<drone>/detection/results  (DetectionResult, pixel bbox)
                |
                |  + TF earth <- camera optical frame, at the DETECTION's own
                |    capture stamp (detection_node deliberately carries the
                |    camera frame's stamp forward for exactly this reason)
                |  + CameraInfo intrinsics
                v
    /<drone>/geotag/survivors   (SurvivorTag, lat/lon)

Cross-drone reconciliation is NOT done here -- two drones seeing the same
person still publish two tags, with unrelated ids, because a single drone
cannot know that. survivor_aggregator_node merges them.
"""

import math
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from sensor_msgs.msg import CameraInfo

from nidar_msgs.msg import DetectionResult, SurvivorTag
from nidar_mission_manager import geo_utils, world_config
from nidar_mission_manager.survivor_aggregator import (
    DEFAULT_LOCAL_DEDUP_RADIUS_M, SurvivorRegistry)

from nidar_geotag import projection

WORLD_CONFIG_PATH = Path('config/world_swarm.yaml')

#: ByteTrack's "no identity yet" sentinel, mirroring nidar_detection's own.
UNTRACKED = -1


class GeotagNode(Node):

    def __init__(self):
        super().__init__('geotag')

        self.declare_parameter('drone_id', 'drone0')
        self.declare_parameter('earth_frame', 'earth')
        # Height of the ground plane in the earth frame. The arena is flat and
        # the state estimator's origin sits on it, so 0.0 -- but a real site
        # with the launch pad on a rise needs this, and a wrong value biases
        # every tag radially outward from the drone rather than uniformly.
        self.declare_parameter('ground_altitude_m', 0.0)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('local_dedup_radius_m', DEFAULT_LOCAL_DEDUP_RADIUS_M)
        self.declare_parameter('camera_info_topic', '')
        # Above this the detection is discarded as a projection artefact: a
        # bbox near the frame edge of a slightly-tilted camera produces a ray
        # that grazes the ground hundreds of metres away. The camera footprint
        # at survey altitude is ~9 m across, so anything beyond a few tens of
        # metres from the drone did not come from this frame.
        self.declare_parameter('max_ground_range_m', 60.0)

        self.drone_id = str(self.get_parameter('drone_id').value)
        self.earth_frame = str(self.get_parameter('earth_frame').value)
        self.ground_z = float(self.get_parameter('ground_altitude_m').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.max_range = float(self.get_parameter('max_ground_range_m').value)

        self.origin = world_config.load_origin(WORLD_CONFIG_PATH)

        self.registry = SurvivorRegistry(
            dedup_radius_m=float(self.get_parameter('local_dedup_radius_m').value))

        self._intrinsics: Optional[tuple] = None   # (fx, fy, cx, cy)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.tag_pub = self.create_publisher(
            SurvivorTag, f'/{self.drone_id}/geotag/survivors', 10)

        cam_topic = (str(self.get_parameter('camera_info_topic').value)
                     or f'/{self.drone_id}/sensor_measurements/gimbal/camera/camera_info')
        self.create_subscription(CameraInfo, cam_topic, self._on_camera_info,
                                 QoSProfile(depth=1,
                                            reliability=ReliabilityPolicy.RELIABLE,
                                            history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(DetectionResult,
                                 f'/{self.drone_id}/detection/results',
                                 self._on_detection, 10)

        # Counters, reported in the periodic summary below. Without these the
        # only visible symptom of a systematically-failing projection is an
        # empty map, which looks identical to "no survivors present".
        self._stats = {'seen': 0, 'low_conf': 0, 'no_tf': 0,
                       'no_intrinsics': 0, 'no_ground_hit': 0,
                       'out_of_range': 0, 'published': 0}
        self.create_timer(10.0, self._log_stats)

        self.get_logger().info(
            f'geotag ready | drone={self.drone_id} | earth_frame={self.earth_frame} '
            f'| ground_z={self.ground_z:.2f}m | min_conf={self.min_confidence} '
            f'| dedup={self.registry.dedup_radius_m:.1f}m | origin={self.origin}')

    # ------------------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo):
        # k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        if fx == 0.0 or fy == 0.0:
            return
        if self._intrinsics is None:
            self.get_logger().info(
                f'camera intrinsics: fx={fx:.2f} fy={fy:.2f} '
                f'cx={cx:.2f} cy={cy:.2f} ({msg.width}x{msg.height})')
        self._intrinsics = (fx, fy, cx, cy)

    @staticmethod
    def _strip_slash(frame: str) -> str:
        """TF2 frame ids carry no leading slash, but this sim's CameraInfo and
        image headers do ('/drone0/gimbal/.../optical_frame'). Looking that up
        verbatim fails with 'frame does not exist' -- which reads exactly like
        a missing transform rather than a string mismatch."""
        return frame[1:] if frame.startswith('/') else frame

    def _on_detection(self, msg: DetectionResult):
        self._stats['seen'] += 1

        if msg.confidence < self.min_confidence:
            self._stats['low_conf'] += 1
            return
        if self._intrinsics is None:
            self._stats['no_intrinsics'] += 1
            return

        camera_frame = self._strip_slash(msg.header.frame_id)
        try:
            # The detection's own capture stamp, not now(): at 8 FPS on CPU
            # the drone has moved appreciably between capture and this
            # callback, and pairing a bbox with a later pose smears the tag
            # along the flight path.
            tf = self.tf_buffer.lookup_transform(
                self.earth_frame, camera_frame, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
            self._stats['no_tf'] += 1
            self.get_logger().debug(f'no transform {self.earth_frame} <- {camera_frame}: {e}')
            return

        t, q = tf.transform.translation, tf.transform.rotation
        fx, fy, cx, cy = self._intrinsics
        u = msg.bbox_x + msg.bbox_w / 2.0
        v = msg.bbox_y + msg.bbox_h / 2.0

        hit = projection.project_pixel_to_ground(
            u, v, fx, fy, cx, cy,
            (t.x, t.y, t.z), (q.x, q.y, q.z, q.w), self.ground_z)
        if hit is None:
            self._stats['no_ground_hit'] += 1
            return

        ground_range = math.hypot(hit[0] - t.x, hit[1] - t.y)
        if ground_range > self.max_range:
            self._stats['out_of_range'] += 1
            return

        lat, lon = geo_utils.enu_to_latlon([(hit[0], hit[1])], *self.origin)[0]

        # A ByteTrack id is per-drone but stable, so it is a definitive merge
        # key within this node. Untracked detections fall back to the radius.
        key = (f'{self.drone_id}:{msg.track_id}'
               if msg.track_id != UNTRACKED else None)
        record, is_new = self.registry.observe(
            lat, lon, self.ground_z, msg.confidence, self.drone_id, key=key)

        tag = SurvivorTag()
        tag.header.stamp = msg.header.stamp
        tag.header.frame_id = self.earth_frame
        tag.survivor_id = record.survivor_id
        tag.latitude = record.latitude
        tag.longitude = record.longitude
        tag.altitude = record.altitude
        tag.confidence = record.confidence
        tag.detecting_drone_id = self.drone_id
        tag.delivery_assigned = False
        tag.delivery_complete = False
        self.tag_pub.publish(tag)
        self._stats['published'] += 1

        if is_new:
            self.get_logger().info(
                f'survivor {record.survivor_id} at ({lat:.7f},{lon:.7f}) '
                f'conf {msg.confidence:.2f} | {ground_range:.1f}m from drone, '
                f'alt {t.z:.1f}m, track {msg.track_id}')

    def _log_stats(self):
        s = self._stats
        if s['seen'] == 0:
            return
        self.get_logger().info(
            f"geotag: {s['seen']} detections -> {s['published']} tags, "
            f"{len(self.registry)} unique | dropped: low_conf={s['low_conf']} "
            f"no_tf={s['no_tf']} no_intrinsics={s['no_intrinsics']} "
            f"no_ground={s['no_ground_hit']} out_of_range={s['out_of_range']}")


def main():
    rclpy.init()
    node = GeotagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
