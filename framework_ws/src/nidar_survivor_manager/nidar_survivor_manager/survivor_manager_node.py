#!/usr/bin/env python3
"""Gazebo survivor-dummy inventory and spawn/remove, independent of flight.

Owns the answer to "which human models are in the world right now", covering
BOTH sources:

  * **Pre-placed** -- the survivor actors listed in world_swarm.yaml's
    `objects:` block (regenerated from survivors.yaml by
    utils/sync_survivors.py) and spawned by AS2 at launch.
  * **Runtime** -- dummies added later through /nidar/survivor_command.

This node used to track only the runtime ones, and that was the whole of a
reported bug: the GCS showed 6 survivors while Gazebo contained 20. The 14
pre-placed actors were invisible to the operator, could not be removed
("unknown survivor id"), and were not cleared by "clear" -- yet the detector
saw them, so the detection count looked inflated against a survivor list
that was itself wrong. Pre-placed actors are ordinary Gazebo models with
ordinary names; there was never a reason they could not be managed here.

Removing a pre-placed survivor removes it from the RUNNING world only.
world_swarm.yaml is the arena definition and is deliberately not rewritten,
so the next launch spawns all of them again. Editing survivors.yaml and
re-running utils/sync_survivors.py is what changes the arena permanently.

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

#: model_type values in world_swarm.yaml's `objects:` that are human dummies.
#: Anything else there (props, obstacles) is left alone -- this node manages
#: survivors, not every object in the world.
SURVIVOR_MODEL_TYPES = ('survivor', 'survivor_actor')

#: How often the full inventory is republished. Publishing only on change is
#: not enough: the pre-placed survivors are registered at startup, before the
#: GCS browser has connected, so a change-only publisher means the operator
#: never receives them and the list looks empty of everything they did not
#: add themselves. Same failure the geotag aggregator had.
LIST_REPUBLISH_PERIOD_SEC = 2.0


class SurvivorManager(Node):

    def __init__(self):
        super().__init__('survivor_manager')
        self.survivors_list_pub = self.create_publisher(String, '/nidar/survivors_list', 10)
        self.survivor_status_pub = self.create_publisher(String, '/nidar/survivor_status', 10)
        self.create_subscription(SurvivorCommand, '/nidar/survivor_command', self._on_survivor_command, 10)

        self.origin = world_config.load_origin(WORLD_CONFIG_PATH)
        self.world_name = world_config.load_world_name(WORLD_CONFIG_PATH)

        # Every survivor model believed to be in the world (id -> (lat, lon)),
        # pre-placed and runtime alike. Keyed by Gazebo model name, which is
        # what the remove service needs.
        self._survivors: dict[str, tuple[float, float]] = {}
        # Which of those came from the world config, so logs and the eventual
        # respawn-on-restart behaviour can be explained accurately.
        self._preplaced: set[str] = set()
        self._survivor_counter = 0

        self._register_preplaced_survivors()

        self.create_timer(LIST_REPUBLISH_PERIOD_SEC, self._publish_survivors_list)

        self.get_logger().info(
            f'survivor_manager ready | {len(self._preplaced)} pre-placed + '
            f'{len(self._survivors) - len(self._preplaced)} runtime survivor(s) tracked')

    def _register_preplaced_survivors(self):
        """Record the survivor actors the world config spawns at launch.

        Their positions are stored in the config as local ENU metres; the GCS
        speaks lat/lon, so convert once here using the same origin every other
        node uses."""
        try:
            objects = world_config.load_world_objects(WORLD_CONFIG_PATH)
        except Exception as e:  # noqa: BLE001 - a malformed world file must not stop runtime spawning
            self.get_logger().warn(f'could not read world objects: {e}')
            return

        for obj in objects:
            if obj.get('model_type') not in SURVIVOR_MODEL_TYPES:
                continue
            name = obj.get('model_name')
            xyz = obj.get('xyz')
            if not name or not xyz or len(xyz) < 2:
                continue
            (lat, lon), = geo_utils.enu_to_latlon(
                [(float(xyz[0]), float(xyz[1]))], *self.origin)
            self._survivors[name] = (lat, lon)
            self._preplaced.add(name)

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
            # Everything in the world, not just what this session added. An
            # operator clearing survivors expects an empty arena; leaving 14
            # pre-placed actors standing was the reported bug.
            for survivor_id in list(self._survivors):
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
            self._survivors[survivor_id] = (lat, lon)
            self._publish_survivors_list()
        self._publish_survivor_status(survivor_id, 'add', ok, detail)

    def _remove_survivor(self, survivor_id: str):
        if not survivor_id or survivor_id not in self._survivors:
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
            # Read the flag BEFORE discarding it, or the message is dead code.
            if survivor_id in self._preplaced and detail == 'ok':
                detail = 'ok (pre-placed; respawns on next sim launch)'
            del self._survivors[survivor_id]
            self._preplaced.discard(survivor_id)
            self._publish_survivors_list()
        self._publish_survivor_status(survivor_id, 'remove', ok, detail)

    def _publish_survivors_list(self):
        payload = {sid: [lat, lon] for sid, (lat, lon) in self._survivors.items()}
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
