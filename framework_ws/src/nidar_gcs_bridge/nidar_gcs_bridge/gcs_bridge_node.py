#!/usr/bin/env python3
"""Translates between the GCS's JSON-over-std_msgs/String rosbridge topics
and NIDAR's typed internal ROS 2 messages.

This is the *only* node that knows the GCS wire format — every other NIDAR
node talks typed messages on /nidar/* topics. See
DOCUMENTS/standard_implementation_plan_ros2_framework.md for why
mission_file_executor.py (which used to do this plus flight execution plus
survivor spawning, all in one node) was split into nidar_gcs_bridge,
nidar_mission_executor, and nidar_survivor_manager.

Optional-field translation note: nidar_msgs' typed command messages have no
concept of "field not provided" (a float32/float64 defaults to 0.0, not
None). Two sentinels carry that distinction across the JSON -> typed
boundary, matching how the two downstream nodes already interpret them:
  - DroneCommand.altitude_m: 0.0 means "not specified" (0.0 is never a
    valid takeoff altitude anyway) -- nidar_mission_executor falls back to
    its own default in that case, same as the original code's
    `cmd.get('altitude_m', DEFAULT)` did for a missing dict key.
  - SurvivorCommand.latitude/longitude: NaN means "not specified" -- no
    real lat/lon is ever NaN, unlike 0.0 which is a plausible (if wrong)
    coordinate. nidar_survivor_manager checks for it exactly where the
    original code checked `lat is None or lon is None`.
"""

import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from nidar_msgs.msg import (
    DetectionResult, DroneCommand, MissionCommand, MissionStatus, SurvivorCommand,
    ZoneAllocation)


class GcsBridge(Node):

    def __init__(self):
        super().__init__('gcs_bridge')

        self.declare_parameter('drone_ids', ['drone0', 'drone1', 'drone2', 'drone3'])

        # --- Outbound to internal NIDAR nodes -----------------------------
        self.mission_command_pub = self.create_publisher(MissionCommand, '/nidar/mission_command', 10)
        self.drone_command_pub = self.create_publisher(DroneCommand, '/nidar/drone_command', 10)
        self.survivor_command_pub = self.create_publisher(SurvivorCommand, '/nidar/survivor_command', 10)

        # --- Outbound to GCS (relays + this node's own early-reject errors) ---
        self.gcs_status_pub = self.create_publisher(String, '/gcs/mission_status', 10)
        self.gcs_zone_pub = self.create_publisher(ZoneAllocation, '/gcs/mission/zone_allocation', 10)
        self.gcs_progress_pub = self.create_publisher(MissionStatus, '/gcs/mission/progress', 10)
        self.gcs_detections_pub = self.create_publisher(DetectionResult, '/gcs/detections', 10)
        self.gcs_paths_pub = self.create_publisher(String, '/gcs/mission/planned_paths', 10)
        self.gcs_drone_status_pub = self.create_publisher(String, '/gcs/drone_control/status', 10)
        self.gcs_survivor_status_pub = self.create_publisher(String, '/gcs/survivor_control/status', 10)
        self.gcs_survivors_list_pub = self.create_publisher(String, '/gcs/survivors/list', 10)

        # --- Inbound from GCS -----------------------------------------------
        self.create_subscription(String, '/gcs/mission_load', self._on_mission_load, 10)
        self.create_subscription(String, '/gcs/mission_start', self._on_mission_start, 10)
        self.create_subscription(String, '/gcs/drone_control/command', self._on_drone_command, 10)
        self.create_subscription(String, '/gcs/survivor_control/command', self._on_survivor_command, 10)

        # --- Inbound from internal NIDAR nodes, relayed to the GCS verbatim ---
        self.create_subscription(String, '/nidar/mission_status', self._relay(self.gcs_status_pub), 10)
        self.create_subscription(ZoneAllocation, '/nidar/zone_allocation', self._relay(self.gcs_zone_pub), 10)
        self.create_subscription(MissionStatus, '/nidar/mission_progress', self._relay(self.gcs_progress_pub), 10)

        # Detections are published per drone by nidar_detection (one node per
        # drone, on its own namespace), so this fans them into the single
        # topic the GCS subscribes to. Merging here rather than in the
        # detection nodes keeps each of those unaware of the GCS entirely --
        # on hardware they run on separate Jetsons. Cross-drone
        # de-duplication of the same physical survivor is deliberately NOT
        # done here: that needs geographic positions, which only exist after
        # Phase 3's geotag node, and it is that phase's survivor aggregator
        # that owns it.
        for ns in (self.get_parameter('drone_ids').value or []):
            self.create_subscription(
                DetectionResult, f'/{ns}/detection/results',
                self._relay(self.gcs_detections_pub), 10)
        self.create_subscription(String, '/nidar/planned_paths', self._relay(self.gcs_paths_pub), 10)
        self.create_subscription(String, '/nidar/drone_control_status', self._relay(self.gcs_drone_status_pub), 10)
        self.create_subscription(String, '/nidar/survivor_status', self._relay(self.gcs_survivor_status_pub), 10)
        self.create_subscription(String, '/nidar/survivors_list', self._relay(self.gcs_survivors_list_pub), 10)

        self.get_logger().info(
            'gcs_bridge ready | relaying detections for '
            f"{list(self.get_parameter('drone_ids').value or [])}")

    @staticmethod
    def _relay(publisher):
        """A subscription callback that republishes the message unchanged."""
        def _cb(msg):
            publisher.publish(msg)
        return _cb

    # ------------------------------------------------------------------
    # Mission load/start -> MissionCommand
    # ------------------------------------------------------------------

    def _on_mission_load(self, msg: String):
        try:
            json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._publish_gcs_status_error(f'invalid mission JSON: {e}')
            return
        self.mission_command_pub.publish(
            MissionCommand(action=MissionCommand.LOAD, mission_json=msg.data))

    def _on_mission_start(self, _msg: String):
        self.mission_command_pub.publish(MissionCommand(action=MissionCommand.START))

    def _publish_gcs_status_error(self, detail: str):
        self.gcs_status_pub.publish(String(data=json.dumps({'state': 'error', 'detail': detail})))

    # ------------------------------------------------------------------
    # Manual drone control -> DroneCommand
    # ------------------------------------------------------------------

    def _on_drone_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._publish_drone_status_error(f'invalid command JSON: {e}')
            return
        altitude_m = cmd.get('altitude_m')
        lat, lon = cmd.get('lat'), cmd.get('lon')
        self.drone_command_pub.publish(DroneCommand(
            drone_id=str(cmd.get('drone_id', '')),
            action=str(cmd.get('action', '')),
            altitude_m=float(altitude_m) if isinstance(altitude_m, (int, float)) else 0.0,
            latitude=float(lat) if isinstance(lat, (int, float)) else math.nan,
            longitude=float(lon) if isinstance(lon, (int, float)) else math.nan,
        ))

    def _publish_drone_status_error(self, detail: str):
        payload = {'drone_id': '?', 'action': '?', 'success': False, 'detail': detail}
        self.gcs_drone_status_pub.publish(String(data=json.dumps(payload)))

    # ------------------------------------------------------------------
    # Survivor control -> SurvivorCommand
    # ------------------------------------------------------------------

    def _on_survivor_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._publish_survivor_status_error(f'invalid command JSON: {e}')
            return
        lat, lon = cmd.get('lat'), cmd.get('lon')
        self.survivor_command_pub.publish(SurvivorCommand(
            action=str(cmd.get('action', '')),
            survivor_id=str(cmd.get('survivor_id', '')),
            latitude=float(lat) if isinstance(lat, (int, float)) else math.nan,
            longitude=float(lon) if isinstance(lon, (int, float)) else math.nan,
        ))

    def _publish_survivor_status_error(self, detail: str):
        payload = {'survivor_id': '?', 'action': '?', 'success': False, 'detail': detail}
        self.gcs_survivor_status_pub.publish(String(data=json.dumps(payload)))


def main():
    # Ctrl+C / a supervisor shutdown otherwise surfaces as a raw
    # ExternalShutdownException traceback in gcs_bridge.log, which reads like
    # a crash when scanning the logs after a failed run and buries whatever
    # the real error was.
    rclpy.init()
    node = GcsBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
