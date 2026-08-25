#!/usr/bin/env python3
"""Generic multi-drone mission executor driven by a JSON mission file.

Unlike mission.py/mission_swarm.py (which hardcode their flight paths), this
node takes its waypoints from a mission file loaded by the GCS at runtime, so
the GCS can drive an arbitrary mission with no manual waypoint entry — see
DOCUMENTS/mission_file_schema.md for the file format and rationale.

Flow, matching the same pattern as trees/square.xml's WaitForEvent node:
  1. GCS publishes the parsed mission JSON to /gcs/mission_load (std_msgs/String).
  2. This node stores it and waits.
  3. GCS publishes any message to /gcs/mission_start (std_msgs/String) — the one
     manual trigger the mission brief permits ("mission start" is not manual
     intervention).
  4. This node arms, takes off, and flies each drone's waypoint list from the
     mission file, then lands — fully autonomously from there.
  5. Progress is published to /gcs/mission_status (std_msgs/String, JSON) so the
     GCS can display it.

Two mission shapes are supported, dispatched on in _run_mission:
  - Per-drone JSON waypoints (mission['drones']) — the original Phase 0 flow,
    sequential GoTo per waypoint. Untouched by Phase 1 (_run_json_waypoint_mission).
  - Boundary-only (mission['boundary'], no 'drones') — Phase 1's auto coverage
    flow: split the boundary into per-drone zones (nidar_mission_manager.
    zone_splitter), generate a lawnmower path per zone (path_planner), publish
    both for the GCS map, then fly each drone's path via FollowPath.

Separately, /gcs/drone_control/command lets an operator manually arm/disarm/
takeoff individual drones (or all) outside of a full mission run — for
pre-flight checks and bench testing. This is a dev/test facility, not part of
the competition-facing Start/Abort/Recall flow (see the implementation plan's
own Phase 9.3 note on the 3-button competition limit); gate it off before the
Phase 7 competition-ready GCS build.
"""

import json
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from geographic_msgs.msg import GeoPoint
from rclpy.node import Node
from std_msgs.msg import String

from as2_python_api.drone_interface_gps import DroneInterfaceGPS
from nidar_msgs.msg import ZoneAllocation

from nidar_mission_manager import geo_utils, path_planner, zone_splitter
from utils.get_drones import get_drones_namespaces

WORLD_CONFIG_PATH = Path('config/world_swarm.yaml')
SURVIVOR_SDF_PATH = Path('models/survivor_actor/survivor_actor.sdf').resolve()


class MissionMonitor(Node):
    """Owns the GCS-facing topics; hands off to executor threads."""

    def __init__(self):
        super().__init__('mission_file_executor')
        self.mission: Optional[dict] = None
        self.status_pub = self.create_publisher(String, '/gcs/mission_status', 10)
        self.zone_pub = self.create_publisher(ZoneAllocation, '/gcs/mission/zone_allocation', 10)
        self.planned_paths_pub = self.create_publisher(String, '/gcs/mission/planned_paths', 10)
        self.control_status_pub = self.create_publisher(String, '/gcs/drone_control/status', 10)
        self.survivor_status_pub = self.create_publisher(String, '/gcs/survivor_control/status', 10)
        self.survivors_list_pub = self.create_publisher(String, '/gcs/survivors/list', 10)
        self.create_subscription(String, '/gcs/mission_load', self._on_load, 10)
        self.create_subscription(String, '/gcs/mission_start', self._on_start, 10)
        self.create_subscription(String, '/gcs/drone_control/command', self._on_drone_command, 10)
        self.create_subscription(String, '/gcs/survivor_control/command', self._on_survivor_command, 10)
        self._executing = False

        self.origin = self._load_origin()
        self.world_name = self._load_world_name()
        self.known_namespaces = get_drones_namespaces(WORLD_CONFIG_PATH)

        # Runtime-spawned survivors only (id -> (lat, lon)) -- NOT the
        # pre-configured survivor_0/1/2 from survivors.yaml, which are spawned
        # at launch time the usual way and never touched here. Spawn/remove
        # go through Gazebo's own native CLI tools (see _add_survivor/
        # _remove_survivor), not a ROS2 service bridge -- ros_gz_bridge's
        # *generic* service bridging (tried first) reliably advertised
        # /world/<world>/create and /world/<world>/remove as discoverable
        # (ros2 service list saw them fine) but rclpy client-side
        # wait_for_service never matched them even after a full 60s wait,
        # on this exact ROS2 Humble + Gazebo Harmonic combination -- a real
        # QoS/type-hash gap in that bridging path, not a timing issue. gz sim's
        # UserCommands services (the same ones AS2's own launch-time object
        # spawning already calls, via `ros2 run ros_gz_sim create` / the
        # native `gz service` CLI) are the proven-reliable path used
        # throughout this whole project already -- confirmed live: instant
        # success, no discovery gap.
        self._runtime_survivors: dict[str, tuple[float, float]] = {}
        self._survivor_counter = 0

        # Persistent, lazily-created drone interfaces shared by both the
        # mission-execution flow and manual drone-control commands, so
        # "manually arm drone0, verify it's healthy, then Start" works and
        # a mission never fights a separate manual-control interface for the
        # same physical/sim drone. Interfaces live for this node's process
        # lifetime (destroy_node cleans them up), not scoped to one mission.
        self._interfaces: dict[str, DroneInterfaceGPS] = {}
        self._interfaces_lock = threading.Lock()

        self._publish_status('idle', 'waiting for mission file')

    def _load_origin(self) -> tuple[float, float, float]:
        with open(WORLD_CONFIG_PATH, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        o = cfg['origin']
        return o['latitude'], o['longitude'], o['altitude']

    def _load_world_name(self) -> str:
        with open(WORLD_CONFIG_PATH, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        return cfg['world_name']

    def _get_interface(self, ns: str) -> DroneInterfaceGPS:
        with self._interfaces_lock:
            if ns not in self._interfaces:
                self._interfaces[ns] = DroneInterfaceGPS(ns, use_sim_time=True, verbose=False)
            return self._interfaces[ns]

    def destroy_node(self):
        with self._interfaces_lock:
            for drone in self._interfaces.values():
                drone.shutdown()
            self._interfaces.clear()
        super().destroy_node()

    CALL_TIMEOUT_SEC = 8.0
    CALL_MAX_ATTEMPTS = 3
    CALL_RETRY_BACKOFF_SEC = 1.5

    def _call_bounded(self, fn, timeout_sec: float = CALL_TIMEOUT_SEC,
                       attempts: int = CALL_MAX_ATTEMPTS) -> bool:
        """Run a blocking AS2 call (drone.arm(), .offboard(), .takeoff(), ...)
        with a hard wall-clock bound, retrying on failure.

        as2_python_api's ServiceHandler has no timeout on its underlying
        synchronous rclpy Client.call(): if a service isn't discovered within
        its own soft 3s wait_for_service check, it logs an error but still
        makes the blocking call anyway -- which can then hang forever with no
        way to recover short of killing this whole process. Running each
        attempt on its own daemon thread and giving up after timeout_sec lets
        us report failure and move on instead of freezing; the orphaned
        thread is a harmless leak for this node's process lifetime.

        Separately: confirmed live that a freshly-constructed drone
        interface's first call frequently fails this same discovery check
        even when nothing is actually wrong -- a second attempt on the same
        (now-registered) interface routinely succeeds. Retrying here means
        the GCS operator doesn't have to notice a failure and click the
        button again themselves. A call that fails because the drone is
        already in the requested state (AS2's own "already armed" guard,
        which returns success=False) gets retried too since a single failure
        can't be told apart from a real one here -- harmless, since a retry
        on an already-discovered service resolves in well under a second,
        not the full timeout.
        """
        for attempt in range(1, attempts + 1):
            result: dict = {}

            def _run():
                try:
                    result['value'] = fn()
                except Exception as e:  # noqa: BLE001 - captured and re-raised on the caller's thread
                    result['error'] = e

            t = threading.Thread(target=_run, daemon=True)
            t.start()
            t.join(timeout=timeout_sec)
            if not t.is_alive():
                if 'error' in result:
                    raise result['error']
                if result.get('value'):
                    return True
            if attempt < attempts:
                time.sleep(self.CALL_RETRY_BACKOFF_SEC)
        return False

    def _on_load(self, msg: String):
        try:
            self.mission = json.loads(msg.data)
            self._publish_status('loaded', f"mission '{self.mission.get('mission_name', '?')}' loaded")
        except json.JSONDecodeError as e:
            self._publish_status('error', f'invalid mission JSON: {e}')

    def _on_start(self, _msg: String):
        if self.mission is None:
            self._publish_status('error', 'start requested but no mission loaded')
            return
        if self._executing:
            self._publish_status('error', 'mission already running')
            return
        self._executing = True
        threading.Thread(target=self._run_mission, args=(self.mission,), daemon=True).start()

    def _publish_status(self, state: str, detail: str, extra: Optional[dict] = None):
        payload = {'state': state, 'detail': detail}
        if extra:
            payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'[{state}] {detail}')

    # ------------------------------------------------------------------
    # Mission dispatch
    # ------------------------------------------------------------------

    def _run_mission(self, mission: dict):
        drones_waypoints = mission.get('drones') or {}
        if drones_waypoints:
            self._run_json_waypoint_mission(mission, drones_waypoints)
            return
        boundary = mission.get('boundary')
        if not boundary or len(boundary) < 3:
            self._publish_status('error', 'mission has neither "drones" nor a valid "boundary"')
            self._executing = False
            return
        self._run_boundary_coverage_mission(mission, boundary)

    def _run_json_waypoint_mission(self, mission: dict, drones_waypoints: dict):
        """Original Phase 0 flow: sequential GoTo per pre-supplied waypoint list.
        Unchanged logic from before Phase 1 — only the interface lifecycle
        changed (shared/persistent via _get_interface instead of a local dict
        that got shut down after every run)."""
        altitude = mission.get('altitude_m', 1.0)
        speed = mission.get('speed_mps', 0.5)
        namespaces = list(drones_waypoints.keys())

        self._publish_status('starting', f'arming {len(namespaces)} drone(s)', {'drones': namespaces})

        try:
            interfaces = {ns: self._get_interface(ns) for ns in namespaces}

            for ns, drone in interfaces.items():
                if not self._call_bounded(drone.arm) or not self._call_bounded(drone.offboard):
                    self._publish_status('error', f'{ns} failed to arm/offboard')
                    return

            self._publish_status('taking_off', 'all drones taking off')
            for ns, drone in interfaces.items():
                if not self._call_bounded(lambda d=drone: d.takeoff(height=altitude, speed=speed)):
                    self._publish_status('error', f'{ns} takeoff failed')
                    return

            max_len = max(len(wps) for wps in drones_waypoints.values())
            for i in range(max_len):
                for ns, drone in interfaces.items():
                    wps = drones_waypoints[ns]
                    if i >= len(wps):
                        continue
                    lat, lon = wps[i]
                    drone.go_to.go_to_gps_point([lat, lon, altitude], speed=speed)
                self._publish_status('running', f'waypoint {i + 1}/{max_len}', {'waypoint_index': i})

            self._publish_status('landing', 'landing all drones')
            for ns, drone in interfaces.items():
                self._call_bounded(lambda d=drone: d.land(speed=0.4))

            self._publish_status('complete', 'mission finished')
        except Exception as e:  # noqa: BLE001 - surface any failure to the GCS instead of dying silently
            self._publish_status('error', f'mission crashed: {e}')
        finally:
            self._executing = False

    def _run_boundary_coverage_mission(self, mission: dict, boundary: list):
        """Phase 1 flow: auto zone-split the boundary, generate a lawnmower
        path per zone, publish both for the GCS map, then fly all drones'
        paths concurrently via FollowPath."""
        speed = mission.get('speed_mps', 0.5)
        # NOT the JSON-waypoint path's 1.0m default -- that would silently
        # conflict with path_planner's swath-width math, which assumes a
        # realistic scan altitude.
        scan_altitude = mission.get('altitude_m', path_planner.DEFAULT_SCAN_ALTITUDE_M)
        namespaces = self.known_namespaces

        self._publish_status('starting', f'splitting boundary into {len(namespaces)} zones',
                              {'drones': namespaces})
        # Check if mission specifies a custom home launch site
        home = mission.get('home')
        origin_lat = home[0] if (home and isinstance(home, (list, tuple)) and len(home) >= 2) else self.origin[0]
        origin_lon = home[1] if (home and isinstance(home, (list, tuple)) and len(home) >= 2) else self.origin[1]
        origin_alt = self.origin[2]

        try:
            zones_latlon = zone_splitter.split_boundary(
                boundary, num_zones=len(namespaces),
                origin_lat=origin_lat, origin_lon=origin_lon, origin_alt=origin_alt)
        except ValueError as e:
            self._publish_status('error', f'zone splitting failed: {e}')
            self._executing = False
            return

        drone_zone = dict(zip(namespaces, zones_latlon))
        drone_waypoints = {}
        for ns, zone in drone_zone.items():
            try:
                drone_waypoints[ns] = path_planner.generate_lawnmower_path(
                    zone, origin_lat=origin_lat, origin_lon=origin_lon, origin_alt=origin_alt,
                    scan_altitude_m=scan_altitude)
            except ValueError as e:
                self._publish_status('error', f'path planning failed for {ns}: {e}')
                self._executing = False
                return

        # Published before arm/takeoff so the GCS shows zones + planned
        # lawnmower paths immediately on Start, independent of flight progress.
        self._publish_zone_allocations(drone_zone)
        self._publish_planned_paths(drone_waypoints)

        try:
            interfaces = {ns: self._get_interface(ns) for ns in namespaces}

            for ns, drone in interfaces.items():
                if not self._call_bounded(drone.arm) or not self._call_bounded(drone.offboard):
                    self._publish_status('error', f'{ns} failed to arm/offboard')
                    return

            self._publish_status('taking_off', 'all drones taking off')
            for ns, drone in interfaces.items():
                if not self._call_bounded(lambda d=drone: d.takeoff(height=scan_altitude, speed=speed)):
                    self._publish_status('error', f'{ns} takeoff failed')
                    return

            self._publish_status('running', 'flying coverage pattern', {'drones': namespaces})
            # follow_path() defaults to wait=True (blocks until that drone's
            # WHOLE path completes). Calling it in a plain sequential loop
            # here would fly zones one drone at a time -- one thread per
            # drone is what makes this actually parallel, multi-drone
            # coverage instead of defeating the point of zone splitting.
            results: dict[str, bool] = {}

            def _fly(ns: str, drone: DroneInterfaceGPS, wps: list):
                results[ns] = drone.follow_path([[lat, lon, alt] for lat, lon, alt in wps], speed=speed)

            threads = [threading.Thread(target=_fly, args=(ns, interfaces[ns], wps))
                       for ns, wps in drone_waypoints.items()]
            for t in threads:
                t.start()
            for t in threads:
                t.join()

            failed = [ns for ns, ok in results.items() if not ok]
            if failed:
                self._publish_status('error', f'follow_path failed for: {failed}')
                return

            self._publish_status('landing', 'landing all drones')
            for ns, drone in interfaces.items():
                self._call_bounded(lambda d=drone: d.land(speed=0.4))

            self._publish_status('complete', 'mission finished')
        except Exception as e:  # noqa: BLE001 - surface any failure to the GCS instead of dying silently
            self._publish_status('error', f'mission crashed: {e}')
        finally:
            self._executing = False

    def _publish_zone_allocations(self, drone_zone: dict):
        stamp = self.get_clock().now().to_msg()
        for ns, zone in drone_zone.items():
            msg = ZoneAllocation()
            msg.header.stamp = stamp
            msg.header.frame_id = 'map'
            msg.drone_id = ns
            msg.zone_vertices = [GeoPoint(latitude=lat, longitude=lon, altitude=0.0) for lat, lon in zone]
            self.zone_pub.publish(msg)

    def _publish_planned_paths(self, drone_waypoints: dict):
        # Drop altitude, keep [lat,lon][] shape identical to mission.drones so
        # the GCS's existing dashed-Polyline render code needs no new shape
        # handling for auto-generated paths.
        payload = {ns: [[lat, lon] for lat, lon, _alt in wps] for ns, wps in drone_waypoints.items()}
        self.planned_paths_pub.publish(String(data=json.dumps(payload)))

    # ------------------------------------------------------------------
    # Manual drone control (dev/test only -- see module docstring)
    # ------------------------------------------------------------------

    def _on_drone_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._publish_control_status('?', '?', False, f'invalid command JSON: {e}')
            return
        threading.Thread(target=self._execute_drone_command, args=(cmd,), daemon=True).start()

    def _execute_drone_command(self, cmd: dict):
        action = cmd.get('action')
        target = cmd.get('drone_id')
        namespaces = self.known_namespaces if target == 'all' else [target]
        if len(namespaces) > 1:
            # Dispatch each drone's (already-bounded) command on its own
            # thread so "all drones" doesn't serialize their up-to-8s waits
            # -- e.g. Arm All shouldn't take 32s in the worst case just
            # because it's checking 4 drones one at a time.
            threads = [threading.Thread(target=self._execute_drone_command_one, args=(ns, action, cmd))
                       for ns in namespaces]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
            return
        for ns in namespaces:
            self._execute_drone_command_one(ns, action, cmd)

    def _execute_drone_command_one(self, ns: str, action: str, cmd: dict):
        if ns not in self.known_namespaces:
            self._publish_control_status(ns, action, False, f'unknown drone {ns!r}')
            return
        if self._executing:
            self._publish_control_status(ns, action, False, 'mission in progress')
            return
        drone = self._get_interface(ns)
        try:
            if action == 'arm':
                ok = self._call_bounded(drone.arm) and self._call_bounded(drone.offboard)
            elif action == 'disarm':
                ok = self._call_bounded(drone.disarm)
            elif action == 'takeoff':
                altitude = cmd.get('altitude_m', path_planner.DEFAULT_SCAN_ALTITUDE_M)
                ok = self._call_bounded(lambda d=drone: d.takeoff(height=altitude, speed=0.5))
            else:
                self._publish_control_status(ns, action, False, f'unknown action {action!r}')
                return
        except Exception as e:  # noqa: BLE001 - report, don't crash the node
            self._publish_control_status(ns, action, False, f'command crashed: {e}')
            return
        detail = 'ok' if ok else 'failed after 3 retries -- drone may already be in this state, or service never became available'
        self._publish_control_status(ns, action, ok, detail)

    def _publish_control_status(self, drone_id: str, action: str, success: bool, detail: str):
        payload = {'drone_id': drone_id, 'action': action, 'success': success, 'detail': detail}
        self.control_status_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'[drone_control] {drone_id} {action}: {detail}')

    # ------------------------------------------------------------------
    # Survivor dummy placement (runtime Gazebo spawn/remove, no sim restart)
    # ------------------------------------------------------------------

    def _on_survivor_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._publish_survivor_status('?', '?', False, f'invalid command JSON: {e}')
            return
        threading.Thread(target=self._execute_survivor_command, args=(cmd,), daemon=True).start()

    def _execute_survivor_command(self, cmd: dict):
        action = cmd.get('action')
        if action == 'add':
            self._add_survivor(cmd.get('lat'), cmd.get('lon'))
        elif action == 'remove':
            self._remove_survivor(cmd.get('survivor_id'))
        elif action == 'clear':
            for survivor_id in list(self._runtime_survivors):
                self._remove_survivor(survivor_id)
        else:
            self._publish_survivor_status('?', action, False, f'unknown action {action!r}')

    def _add_survivor(self, lat, lon):
        if lat is None or lon is None:
            self._publish_survivor_status('?', 'add', False, 'missing lat/lon')
            return
        survivor_id = f'survivor_runtime_{self._survivor_counter}'
        self._survivor_counter += 1

        # ENU x/y relative to the world origin; z=0.0 external, matching the
        # exact convention launch-time objects: spawning already uses -- the
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

    def _remove_survivor(self, survivor_id):
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
    node = MissionMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
