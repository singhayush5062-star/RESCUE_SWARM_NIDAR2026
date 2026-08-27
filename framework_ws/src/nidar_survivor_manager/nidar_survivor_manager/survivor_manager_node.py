#!/usr/bin/env python3
"""Runtime Gazebo survivor-dummy spawn/remove, independent of flight execution.

Only the pre-configured survivor_0/1/2 from project_gazebo/config/survivors.yaml
are spawned at launch time the usual way (as world_swarm.yaml 'objects:') and
are never touched here -- this node only tracks and manages dummies added at
runtime via /nidar/survivor_command (id -> lat/lon).

Spawn/remove go through Gazebo's own native CLI tools, not a ROS2 service
bridge -- ros_gz_bridge's *generic* service bridging (tried first) reliably
advertised /world/<world>/create and /world/<world>/remove as discoverable
(`ros2 service list` saw them fine) but rclpy client-side wait_for_service
never matched them even after a full 60s wait, on this exact ROS2 Humble +
Gazebo Harmonic combination -- a real QoS/type-hash gap in that bridging
path, not a timing issue. gz sim's UserCommands services (the same ones
AS2's own launch-time object spawning already calls, via
`ros2 run ros_gz_sim create` / the native `gz service` CLI) are the
proven-reliable path used throughout this project already -- confirmed live:
instant success, no discovery gap. See
DOCUMENTS/standard_implementation_plan_ros2_framework.md.
"""

import json
import math
import subprocess
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from nidar_msgs.msg import SurvivorCommand

from nidar_mission_manager import geo_utils, world_config

WORLD_CONFIG_PATH = Path('config/world_swarm.yaml')
SURVIVOR_SDF_PATH = Path('models/survivor_actor/survivor_actor.sdf').resolve()


class SurvivorManager(Node):

    def __init__(self):
        super().__init__('survivor_manager')
        self.survivors_list_pub = self.create_publisher(String, '/nidar/survivors_list', 10)
        self.survivor_status_pub = self.create_publisher(String, '/nidar/survivor_status', 10)
        self.create_subscription(SurvivorCommand, '/nidar/survivor_command', self._on_survivor_command, 10)

        self.origin = world_config.load_origin(WORLD_CONFIG_PATH)
        self.world_name = world_config.load_world_name(WORLD_CONFIG_PATH)

        # Runtime-spawned survivors only (id -> (lat, lon)).
        self._runtime_survivors: dict[str, tuple[float, float]] = {}
        self._survivor_counter = 0

        self.get_logger().info('survivor_manager ready')

    def _on_survivor_command(self, cmd: SurvivorCommand):
        # Spawn/remove block on a subprocess call (up to 10s) -- run off the
        # executor thread so this node keeps processing other callbacks
        # (e.g. a "clear" removing several survivors) instead of stalling.
        threading.Thread(target=self._execute_survivor_command, args=(cmd,), daemon=True).start()

    def _execute_survivor_command(self, cmd: SurvivorCommand):
        if cmd.action == 'add':
            self._add_survivor(cmd.latitude, cmd.longitude)
        elif cmd.action == 'remove':
            self._remove_survivor(cmd.survivor_id)
        elif cmd.action == 'clear':
            for survivor_id in list(self._runtime_survivors):
                self._remove_survivor(survivor_id)
        else:
            self._publish_survivor_status('?', cmd.action, False, f'unknown action {cmd.action!r}')

    def _add_survivor(self, lat: float, lon: float):
        if math.isnan(lat) or math.isnan(lon):
            self._publish_survivor_status('?', 'add', False, 'missing lat/lon')
            return
        survivor_id = f'survivor_runtime_{self._survivor_counter}'
        self._survivor_counter += 1

        # ENU x/y relative to the world origin; z=0.0 external, matching the
        # exact convention launch-time object spawning already uses -- the
        # SDF's own internal <pose> (z=1.0, hip-height skeleton root) is what
        # actually puts the feet on the ground, not this z.
        (x, y), = geo_utils.latlon_to_enu([(lat, lon)], *self.origin)

        try:
            result = subprocess.run(
                ['ros2', 'run', 'ros_gz_sim', 'create',
                 '-world', self.world_name,
                 '-file', str(SURVIVOR_SDF_PATH),
                 '-name', survivor_id,
                 '-x', str(x), '-y', str(y), '-z', '0.0'],
                capture_output=True, text=True, timeout=10.0)
            # ros2 run routes rclpy's [INFO]-level logging (including this
            # tool's own "OK creation of entity." success line) to stderr,
            # not stdout -- confirmed live: checking stdout alone reported
            # every genuinely-successful spawn as a failure. Check both.
            combined_output = result.stdout + result.stderr
            ok = result.returncode == 0 and 'OK creation of entity' in combined_output
            detail = 'ok' if ok else f'spawn failed: {combined_output.strip()}'
        except subprocess.TimeoutExpired:
            ok, detail = False, 'spawn timed out after 10s'

        if ok:
            self._runtime_survivors[survivor_id] = (lat, lon)
            self._publish_survivors_list()
        self._publish_survivor_status(survivor_id, 'add', ok, detail)

    def _remove_survivor(self, survivor_id: str):
        if not survivor_id or survivor_id not in self._runtime_survivors:
            self._publish_survivor_status(survivor_id or '?', 'remove', False, 'unknown survivor id')
            return

        try:
            result = subprocess.run(
                ['gz', 'service', '-s', f'/world/{self.world_name}/remove',
                 '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '3000',
                 '--req', f'name: "{survivor_id}" type: 2'],  # type 2 = MODEL
                capture_output=True, text=True, timeout=5.0)
            combined_output = result.stdout + result.stderr
            ok = result.returncode == 0 and 'true' in combined_output.lower()
            detail = 'ok' if ok else f'delete failed: {combined_output.strip()}'
        except subprocess.TimeoutExpired:
            ok, detail = False, 'delete timed out after 5s'

        if ok:
            del self._runtime_survivors[survivor_id]
            self._publish_survivors_list()
        self._publish_survivor_status(survivor_id, 'remove', ok, detail)

    def _publish_survivors_list(self):
        payload = {sid: [lat, lon] for sid, (lat, lon) in self._runtime_survivors.items()}
        self.survivors_list_pub.publish(String(data=json.dumps(payload)))

    def _publish_survivor_status(self, survivor_id: str, action: str, success: bool, detail: str):
        payload = {'survivor_id': survivor_id, 'action': action, 'success': success, 'detail': detail}
        self.survivor_status_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'[survivor_control] {survivor_id} {action}: {detail}')


def main():
    rclpy.init()
    node = SurvivorManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
