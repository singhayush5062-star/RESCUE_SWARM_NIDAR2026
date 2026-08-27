#!/usr/bin/env python3
"""Multi-drone flight orchestration + manual drone control.

Two mission shapes are supported, dispatched on in _run_mission:
  - Per-drone JSON waypoints (mission['drones']) — the original Phase 0 flow,
    sequential GoTo per waypoint (_run_json_waypoint_mission).
  - Boundary-only (mission['boundary'], no 'drones') — Phase 1's auto coverage
    flow: split the boundary into per-drone zones (nidar_mission_manager.
    zone_splitter), generate a lawnmower path per zone (path_planner), publish
    both for the GCS map, then fly each drone's path via FollowPath
    (_run_boundary_coverage_mission).

Manual arm/disarm/takeoff (via /nidar/drone_command) lives in this same node,
not a separate one, because both flows share one thing that can't be split
across nodes without adding RPC: the persistent, lazily-created
DroneInterfaceGPS pool (_get_interface/_interfaces/_interfaces_lock). A
mission run and a manual "arm drone0" click must resolve to the *same* drone
interface, which is also why _execute_drone_command_one refuses to run while
a mission is executing. See
DOCUMENTS/standard_implementation_plan_ros2_framework.md.

This is a dev/test facility, not part of the competition-facing
Start/Abort/Recall flow (see the implementation plan's own Phase 9.3 note on
the 3-button competition limit); gate it off before the Phase 7
competition-ready GCS build.
"""

import json
import math
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, Pose
from rclpy.node import Node
from std_msgs.msg import String

from as2_python_api.drone_interface_gps import DroneInterfaceGPS
from nidar_msgs.msg import DroneCommand, MissionCommand, ZoneAllocation
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from nidar_mission_manager import geo_utils, path_planner, world_config, zone_splitter

WORLD_CONFIG_PATH = Path('config/world_swarm.yaml')
# 12ft x 12ft: the fixed launch/landing area every drone must launch from and
# land within (competition rule). GCS-configured drone_launch_positions are
# validated against this before teleporting anything.
LAUNCH_BOX_SIZE_M = 3.6576
TELEPORT_SPAWN_HEIGHT_M = 0.2   # matches world_swarm.yaml's own default spawn z


class MissionExecutor(Node):
    """Owns mission dispatch, flight execution, and manual drone control."""

    def __init__(self):
        super().__init__('mission_executor')
        self.mission: Optional[dict] = None
        self.status_pub = self.create_publisher(String, '/nidar/mission_status', 10)
        self.zone_pub = self.create_publisher(ZoneAllocation, '/nidar/zone_allocation', 10)
        self.planned_paths_pub = self.create_publisher(String, '/nidar/planned_paths', 10)
        self.control_status_pub = self.create_publisher(String, '/nidar/drone_control_status', 10)
        self.create_subscription(MissionCommand, '/nidar/mission_command', self._on_mission_command, 10)
        self.create_subscription(DroneCommand, '/nidar/drone_command', self._on_drone_command, 10)
        self._executing = False

        self.origin = world_config.load_origin(WORLD_CONFIG_PATH)
        self.world_name = world_config.load_world_name(WORLD_CONFIG_PATH)
        self.known_namespaces = world_config.get_drones_namespaces(WORLD_CONFIG_PATH)

        # Teleports a drone to a GCS-configured launch position before
        # arming, no sim restart needed -- as2_gazebo_assets' own
        # set_entity_pose_bridge node (world_bridges: [set_entity_pose] in
        # world_swarm.yaml), not the generic ros_gz_bridge service bridging
        # that proved unreliable for spawn/remove earlier in this project.
        self.set_pose_client = self.create_client(SetEntityPose, f'/world/{self.world_name}/set_pose')

        # Persistent, lazily-created drone interfaces shared by both the
        # mission-execution flow and manual drone-control commands, so
        # "manually arm drone0, verify it's healthy, then Start" works and
        # a mission never fights a separate manual-control interface for the
        # same physical/sim drone. Interfaces live for this node's process
        # lifetime (destroy_node cleans them up), not scoped to one mission.
        self._interfaces: dict[str, DroneInterfaceGPS] = {}
        self._interfaces_lock = threading.Lock()
        # ns -> (x, y) earth-frame launch position, filled per mission run
        self._launch_local: dict[str, tuple] = {}
        # Centre of the 12ft launch box from the last loaded mission's `home`,
        # so interactive set_launch_position validates against the same box
        # the operator sees drawn on the GCS map.
        self._launch_box_center = None

        self._publish_status('idle', 'waiting for mission file')

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

    # as2_msgs/msg/PlatformStatus state enum, mirrored here so this node
    # doesn't need the message import just to read an int off drone.info.
    STATE_DISARMED = 0
    STATE_LANDED = 1
    STATE_TAKING_OFF = 2
    STATE_FLYING = 3
    STATE_LANDING = 4
    STATE_NAMES = {-1: 'EMERGENCY', 0: 'DISARMED', 1: 'LANDED',
                   2: 'TAKING_OFF', 3: 'FLYING', 4: 'LANDING'}

    # Generous: a takeoff/land blocks for the whole physical maneuver, and
    # _call_physical_action returns as soon as the target state is actually
    # reached, so a large bound only ever costs time on a genuine failure.
    # 25m (DEFAULT_SCAN_ALTITUDE_M) at a slow 0.5 m/s is already 50s.
    PHYSICAL_ACTION_TIMEOUT_SEC = 120.0

    def _call_physical_action(self, drone: DroneInterfaceGPS, fn, target_states,
                               label: str,
                               timeout_sec: float = PHYSICAL_ACTION_TIMEOUT_SEC) -> bool:
        """Run a blocking AS2 flight action (takeoff/land) and judge success by
        the drone's ACTUAL platform state, not by the call's return value.

        Do NOT use _call_bounded for these. That wrapper exists for quick
        service calls (arm/offboard/set_pose) and does two things that are
        actively wrong for a physical maneuver:

        1. Its 8s bound is far shorter than the maneuver itself. Confirmed
           live from the TakeoffBehavior node's own log: a takeoff to 8m
           completed in 9.8s ("Takeoff end" at t+9.8), but _call_bounded had
           already given up at t+8.0 and reported failure.
        2. It then RETRIES. Takeoff is not idempotent -- the platform FSM had
           by then legitimately advanced ARM->LANDED->TAKING_OFF->FLYING, so
           the retry's TAKE_OFF transition was correctly refused
           ("TakeoffBehavior: Could not set FSM to takeoff" -> "Goal
           Rejected"), and after 3 attempts a fully SUCCESSFUL takeoff was
           reported to the GCS as "drone0 takeoff failed", aborting the
           mission mid-air. Verified independently: /drone0/platform/info
           read state=3 (FLYING) at the same moment the node claimed the
           takeoff had failed.

        So: poll the platform state while the call runs, and succeed the
        moment it reaches target_states. That makes the physical outcome the
        source of truth and removes the retry entirely.
        """
        result: dict = {}

        def _run():
            try:
                result['value'] = fn()
            except Exception as e:  # noqa: BLE001 - surfaced by the caller below
                result['error'] = e

        start_state = drone.info.get('state')
        self.get_logger().info(
            f'[{label}] starting; platform state={self.STATE_NAMES.get(start_state, start_state)}')

        t = threading.Thread(target=_run, daemon=True)
        t.start()

        deadline = time.monotonic() + timeout_sec
        last_state = start_state
        while time.monotonic() < deadline:
            state = drone.info.get('state')
            if state != last_state:
                self.get_logger().info(
                    f'[{label}] platform state -> {self.STATE_NAMES.get(state, state)}')
                last_state = state
            if state in target_states:
                self.get_logger().info(
                    f'[{label}] reached target state '
                    f'{self.STATE_NAMES.get(state, state)} -- success')
                return True
            if not t.is_alive():
                # Call returned without the state ever reaching the target.
                # Give the status topic a brief grace period to catch up
                # before calling it a failure.
                if 'error' in result:
                    self.get_logger().error(f'[{label}] raised: {result["error"]}')
                    return False
                grace = time.monotonic() + 2.0
                while time.monotonic() < grace:
                    if drone.info.get('state') in target_states:
                        self.get_logger().info(f'[{label}] target state reached during grace -- success')
                        return True
                    time.sleep(0.1)
                self.get_logger().error(
                    f'[{label}] call returned {result.get("value")!r} but platform state is '
                    f'{self.STATE_NAMES.get(drone.info.get("state"), "?")}, not target')
                return False
            time.sleep(0.1)

        self.get_logger().error(
            f'[{label}] timed out after {timeout_sec}s; platform state is '
            f'{self.STATE_NAMES.get(drone.info.get("state"), "?")}')
        return False

    ARM_OFFBOARD_CONFIRM_TIMEOUT_SEC = 5.0

    def _wait_armed_offboard(self, drone: DroneInterfaceGPS,
                              timeout_sec: float = ARM_OFFBOARD_CONFIRM_TIMEOUT_SEC) -> bool:
        """Poll the drone's own PlatformInfo until it reports armed+offboard.

        arm()/offboard() returning True only confirms the *service call*
        succeeded -- not that every other subscriber to the platform's status
        topic has processed the resulting state change yet. TakeoffBehavior's
        C++ server has its own independent subscription to that same topic
        and its own FSM gate (sendEventFSME(TAKE_OFF) in takeoff_behavior.cpp);
        if a takeoff goal reaches it before that FSM has caught up, it rejects
        the goal outright ("Goal Rejected" -- not a timeout, an immediate
        rejection, so _call_bounded's retry-with-backoff exhausts its 3
        attempts on the same stale read half the time). Confirmed live:
        takeoff rejected twice in a row immediately following a fresh
        arm+offboard. Waiting for OUR OWN client-side subscription to catch
        up is not a strict guarantee the takeoff server's has too, but it's
        the same topic fanned out by the same publisher, so in practice it
        closes the race instead of guessing with a fixed sleep.
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            info = drone.info
            if info.get('armed') and info.get('offboard'):
                return True
            time.sleep(0.1)
        return False

    GPS_FIX_TIMEOUT_SEC = 5.0

    def _wait_gps_fix(self, drone: DroneInterfaceGPS,
                       timeout_sec: float = GPS_FIX_TIMEOUT_SEC) -> Optional[tuple]:
        """Poll the drone's own GPS module until a real fix arrives.

        drone.gps.pose defaults to [0.0, 0.0, 0.0] until the first
        NavSatFix is received -- reading it immediately after constructing a
        fresh interface can otherwise silently hand back "the drone is at
        (0,0)" instead of its real launch position, corrupting nearest-zone
        assignment for everyone. Returns (lat, lon) or None on timeout.

        math.isfinite is not optional here: a NavSatFix with no lock
        publishes NaN, and NaN fails every equality test, so the obvious
        `fix[0] != 0.0` "is it set yet" guard ACCEPTS NaN as a valid fix.
        Confirmed live -- drone3 was handed (nan, nan) as its launch
        position and told to return-to-launch at "nan,nan".
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            fix = drone.gps.pose
            if (fix and math.isfinite(fix[0]) and math.isfinite(fix[1])
                    and (fix[0] != 0.0 or fix[1] != 0.0)):
                return (fix[0], fix[1])
            time.sleep(0.1)
        return None

    def _inside_launch_box(self, lat: float, lon: float, box_center) -> bool:
        """Is (lat, lon) inside the fixed 12ft x 12ft launch/landing box
        centred on box_center? Shared by mission-start validation and the
        interactive set_launch_position command so both enforce one rule."""
        (dx, dy), = geo_utils.latlon_to_enu([(lat, lon)], box_center[0], box_center[1], self.origin[2])
        half = LAUNCH_BOX_SIZE_M / 2.0
        return abs(dx) <= half and abs(dy) <= half

    def _teleport_drone(self, ns: str, lat: float, lon: float) -> bool:
        """Move drone `ns` to (lat, lon) via as2_gazebo_assets' set_pose
        bridge, before it's armed. Uses the same blocking Client.call() +
        _call_bounded wrapper every other AS2 service call in this file
        goes through -- not call_async()+spin_until_future_complete(), which
        would fight this node's own main-thread rclpy.spin() for control of
        the executor since this runs on a worker thread (see _run_mission's
        threading.Thread caller)."""
        (x, y), = geo_utils.latlon_to_enu([(lat, lon)], *self.origin)
        req = SetEntityPose.Request()
        req.entity = Entity(name=ns, type=Entity.MODEL)
        req.pose = Pose(position=Point(x=x, y=y, z=TELEPORT_SPAWN_HEIGHT_M))

        def _call():
            if not self.set_pose_client.wait_for_service(timeout_sec=3.0):
                return False
            result = self.set_pose_client.call(req)
            return bool(result and result.success)

        return self._call_bounded(_call)

    def _resolve_drone_positions(self, mission: dict, interfaces: dict) -> Optional[dict]:
        """Apply any GCS-configured drone_launch_positions (teleporting each
        listed drone before it's armed), then return every drone's actual
        (lat, lon) -- teleported or default -- for nearest-zone assignment
        and general bookkeeping. Returns None (after publishing an error) if
        a requested position falls outside the fixed 12ft x 12ft launch box,
        or if a teleport itself fails.
        """
        launch_positions = mission.get('drone_launch_positions') or {}
        home = mission.get('home')
        box_center = (home[0], home[1]) if (home and isinstance(home, (list, tuple))
                                             and len(home) >= 2) else (self.origin[0], self.origin[1])

        for ns, pos in launch_positions.items():
            if ns not in interfaces or not isinstance(pos, (list, tuple)) or len(pos) < 2:
                continue
            if not self._inside_launch_box(pos[0], pos[1], box_center):
                self._publish_status(
                    'error',
                    f'{ns} launch position is outside the {LAUNCH_BOX_SIZE_M:.2f}m '
                    f'({LAUNCH_BOX_SIZE_M / 0.3048:.0f}ft) launch box')
                return None
            if not self._teleport_drone(ns, pos[0], pos[1]):
                self._publish_status('error', f'{ns} failed to move to its configured launch position')
                return None

        drone_positions = {}
        self._launch_local = {}
        for ns, drone in interfaces.items():
            fix = self._wait_gps_fix(drone)
            drone_positions[ns] = fix if fix is not None else (self.origin[0], self.origin[1])
            # Earth-frame launch position, recorded for return-to-launch.
            # NOT derived from GPS: go_to_gps_point converts a target lat/lon
            # into a delta from the drone's OWN gps.origin (which
            # set_origin_on_start pins to that drone's own spawn point) and
            # then publishes that delta in the SHARED 'earth' frame. Asking a
            # drone to return to its own launch lat/lon therefore yields delta
            # (0,0) -> earth (0,0) -> the world origin, for every drone.
            # Confirmed live: three drones given three different, correct RTL
            # targets all landed on the world origin ~2.2m from their real
            # launch spots. self_localization/pose is already in the earth
            # frame, so going back to it via go_to_point needs no conversion
            # and no origin indirection.
            pos = drone.position
            if pos and all(math.isfinite(v) for v in pos[:3]):
                self._launch_local[ns] = (float(pos[0]), float(pos[1]))
        return drone_positions

    RTL_TIMEOUT_SEC = 120.0

    def _compensate_gps_path(self, drone, ns: str, waypoints: list,
                             plan_origin: tuple) -> list:
        """Rewrite mission-frame GPS waypoints into the frame AS2 will
        actually interpret them in.

        AS2's FollowPathGpsModule converts each waypoint with
        `geodetic2enu(waypoint, drone.gps.origin)` and then publishes the
        result as an ABSOLUTE point in the shared `earth` frame
        (as2_python_api/behavior_actions/followpath_behavior.py, the
        `isinstance(path, GeoPath)` branch). Because `set_origin_on_start`
        pins every drone's gps.origin to its own spawn fix, that delta is
        measured from a different place for every drone -- so a path planned
        around the mission origin lands shifted by each drone's own spawn
        offset from it. Confirmed live: this is the same frame bug already
        fixed for RTL in _return_to_launch, which was only ever fixed there.

        The fix is to invert AS2's transform before handing the path over:
        express the desired earth-frame position as the lat/lon that sits
        that far from THIS drone's own origin, so AS2's own conversion turns
        it straight back into the position we meant.

        Altitude is passed through untouched -- AS2 deliberately discards the
        converted z and uses the raw waypoint altitude as height above
        origin (its own "CAUTION: using height from origin" comment).

        `plan_origin` must be the origin the path was actually PLANNED
        against, which is the mission's `home` when it sets one and the world
        origin otherwise -- not unconditionally the world origin. Getting
        that wrong reintroduces exactly the offset this method exists to
        remove, just sourced from the launch site instead of the spawn point.
        """
        gps_origin = getattr(drone, 'gps', None) and drone.gps.origin
        if not gps_origin:
            self.get_logger().warn(
                f'[{ns}] gps origin unavailable; sending path uncompensated')
            return [[lat, lon, alt] for lat, lon, alt in waypoints]

        lat0, lon0, h0 = gps_origin
        # Mission frame -> earth ENU, using the same origin the path was
        # planned against.
        enu = geo_utils.latlon_to_enu([(lat, lon) for lat, lon, _ in waypoints],
                                       plan_origin[0], plan_origin[1], plan_origin[2])
        # earth ENU -> lat/lon around THIS drone's origin.
        compensated = geo_utils.enu_to_latlon(enu, lat0, lon0, h0)

        shift = math.hypot(enu[0][0], enu[0][1]) if enu else 0.0
        self.get_logger().info(
            f'[{ns}] path compensated for gps origin '
            f'({lat0:.7f},{lon0:.7f}); first waypoint earth '
            f'({enu[0][0]:.2f},{enu[0][1]:.2f}) |{shift:.1f}m from plan origin')
        return [[c[0], c[1], wp[2]] for c, wp in zip(compensated, waypoints)]

    def _return_to_launch(self, interfaces: dict, drone_positions: dict,
                           altitude_m: float, speed: float):
        """Fly every drone back over its own launch position before landing.

        AS2's land() descends wherever the drone currently is -- at the end
        of a coverage pattern that is the last waypoint of its lawnmower
        path, potentially most of the arena away from where it took off. The
        competition requires every drone to launch from AND land within the
        fixed 12ft x 12ft box, so the return leg has to be explicit.

        Runs one thread per drone (like the coverage legs) so four drones
        transit concurrently instead of serialising. Failures here are
        logged but not fatal: a drone that cannot get home should still be
        landed rather than left hovering until its battery runs out.
        """
        def _go_home(ns: str, drone: DroneInterfaceGPS):
            home = (drone_positions or {}).get(ns)
            if home is None or not (math.isfinite(home[0]) and math.isfinite(home[1])):
                self.get_logger().warn(
                    f'[{ns} rtl] no usable launch position ({home}), skipping return -- '
                    f'it will land where it finished its coverage leg')
                return
            try:
                # DroneInterfaceGPS assigns self.go_to = GoToGpsModule, so the
                # only method available is go_to_gps_point -- there is no
                # go_to_point on this interface (confirmed live:
                # "'GoToGpsModule' object has no attribute 'go_to_point'").
                #
                # go_to_gps_point does: enu = geodetic2enu(target, gps.origin),
                # then publishes that enu as an ABSOLUTE point in the shared
                # 'earth' frame. gps.origin is each drone's OWN spawn fix, so
                # passing a drone its own launch lat/lon yields enu (0,0) ->
                # earth (0,0) -> the world origin, for every drone. That is
                # the bug measured earlier: three drones, three correct
                # targets, all landing on the world origin.
                #
                # Invert it: to arrive at earth-frame (X0, Y0), pass the
                # lat/lon that is (X0, Y0) away FROM this drone's own gps
                # origin. go_to then converts it straight back to (X0, Y0).
                gps_origin = drone.gps.origin
                if not gps_origin:
                    self.get_logger().warn(
                        f'[{ns} rtl] gps origin unavailable, skipping return')
                    return
                lat0, lon0, h0 = gps_origin
                tgt_lat, tgt_lon = geo_utils.enu_to_latlon(
                    [(home[0], home[1])], lat0, lon0, h0)[0]
                self.get_logger().info(
                    f'[{ns} rtl] returning to launch earth=({home[0]:.2f},{home[1]:.2f}) '
                    f'via compensated gps ({tgt_lat:.7f},{tgt_lon:.7f}) at {altitude_m}m')
                drone.go_to.go_to_gps_point([tgt_lat, tgt_lon, altitude_m], speed=speed)
                pos = drone.position
                self.get_logger().info(
                    f'[{ns} rtl] over launch position; now at '
                    f'({pos[0]:.2f},{pos[1]:.2f}) target ({home[0]:.2f},{home[1]:.2f})')
            except Exception as e:  # noqa: BLE001 - land anyway, see docstring
                self.get_logger().error(f'[{ns} rtl] return failed: {e}')

        threads = [threading.Thread(target=_go_home, args=(ns, d), daemon=True)
                   for ns, d in interfaces.items()]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=self.RTL_TIMEOUT_SEC)

    def _disarm_stuck_drones(self, interfaces: dict):
        """Best-effort cleanup after a failed mission.

        A mission that fails partway through (bad takeoff, follow_path
        error, ...) otherwise leaves any already-armed drone armed
        indefinitely -- confirmed live: the *next* mission attempt then
        fails immediately at arm() with AS2's own "already armed" guard
        ("Service returned failure"), for every drone that got that far,
        forcing a manual disarm via the drone control panel before any
        retry can even start. Checking live .info instead of tracking which
        drones this run armed keeps this correct even if a drone was left
        armed by an earlier, unrelated failed attempt.
        """
        for ns, drone in interfaces.items():
            try:
                if drone.info.get('armed'):
                    self._call_bounded(drone.disarm, timeout_sec=3.0, attempts=1)
            except Exception as e:  # noqa: BLE001 - cleanup is best-effort; never mask the real error
                self.get_logger().warn(f'{ns} cleanup disarm failed: {e}')

    # ------------------------------------------------------------------
    # Mission load/start dispatch
    # ------------------------------------------------------------------

    def _on_mission_command(self, msg: MissionCommand):
        if msg.action == MissionCommand.LOAD:
            try:
                self.mission = json.loads(msg.mission_json)
                home = self.mission.get('home')
                self._launch_box_center = (
                    (home[0], home[1])
                    if home and isinstance(home, (list, tuple)) and len(home) >= 2
                    else (self.origin[0], self.origin[1]))
                self._publish_status('loaded', f"mission '{self.mission.get('mission_name', '?')}' loaded")
            except json.JSONDecodeError as e:
                self._publish_status('error', f'invalid mission JSON: {e}')
            return
        if msg.action == MissionCommand.START:
            if self.mission is None:
                self._publish_status('error', 'start requested but no mission loaded')
                return
            if self._executing:
                self._publish_status('error', 'mission already running')
                return
            self._executing = True
            threading.Thread(target=self._run_mission_guarded, args=(self.mission,), daemon=True).start()

    def _publish_status(self, state: str, detail: str, extra: Optional[dict] = None):
        payload = {'state': state, 'detail': detail}
        if extra:
            payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'[{state}] {detail}')

    def _run_mission_guarded(self, mission: dict):
        """Top-level safety net for the mission thread.

        A bare threading.Thread target's own exception is printed to stderr
        by Python's default excepthook and otherwise silently swallowed --
        it never reaches _run_json_waypoint_mission's/
        _run_boundary_coverage_mission's own try/except (those only cover
        part of their body, not everything _run_mission does before
        dispatching to one of them), leaving self._executing stuck True
        forever: every future Start reports "mission already running" and
        every manual drone command reports "mission in progress", with no
        recovery short of restarting this node. Confirmed as a real gap
        while adding nearest-zone-assignment drone/GPS setup ahead of
        _run_boundary_coverage_mission's own try block. This wrapper is a
        pure safety net -- normal success/failure still goes through each
        function's own status publishing and cleanup; this only fires for
        whatever isn't already caught there.
        """
        try:
            self._run_mission(mission)
        except Exception as e:  # noqa: BLE001 - last-resort catch-all, see above
            self._publish_status('error', f'mission crashed: {e}')
        finally:
            self._executing = False

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
        """Original Phase 0 flow: sequential GoTo per pre-supplied waypoint list."""
        # float(...): a whole-number speed/altitude from the GCS (e.g. its
        # own 2.0 m/s default) serializes over JSON as a bare integer --
        # JSON/JS don't distinguish 2 from 2.0 -- so mission.get() can hand
        # back a Python int here. AS2's generated action-goal setters
        # (confirmed for FollowPath.Goal.max_speed) assert strict float
        # typing and raise on an int, so this must be coerced before it
        # reaches any AS2 call.
        altitude = float(mission.get('altitude_m', 1.0))
        speed = float(mission.get('speed_mps', 0.5))
        # Per-drone altitude override from the GCS (namespace -> meters),
        # falling back to the mission-wide altitude for any drone not listed.
        drone_altitudes = {ns: float(a) for ns, a in (mission.get('drone_altitudes') or {}).items()}
        namespaces = list(drones_waypoints.keys())

        self._publish_status('starting', f'arming {len(namespaces)} drone(s)', {'drones': namespaces})

        # Apply any GCS-configured drone_launch_positions (teleport within
        # the fixed 12ft launch box) before arming -- same competition
        # constraint as the boundary-coverage flow, see
        # _resolve_drone_positions.
        interfaces: dict[str, DroneInterfaceGPS] = {ns: self._get_interface(ns) for ns in namespaces}
        drone_positions = self._resolve_drone_positions(mission, interfaces)
        if drone_positions is None:
            self._executing = False
            return

        succeeded = False
        try:
            for ns, drone in interfaces.items():
                if not self._call_bounded(drone.arm) or not self._call_bounded(drone.offboard):
                    self._publish_status('error', f'{ns} failed to arm/offboard')
                    return
                if not self._wait_armed_offboard(drone):
                    self._publish_status('error', f'{ns} armed+offboard state never confirmed')
                    return

            self._publish_status('taking_off', 'all drones taking off')
            for ns, drone in interfaces.items():
                ns_altitude = drone_altitudes.get(ns, altitude)
                if not self._call_physical_action(
                        drone, lambda d=drone, a=ns_altitude: d.takeoff(height=a, speed=speed),
                        (self.STATE_FLYING,), f'{ns} takeoff'):
                    self._publish_status('error', f'{ns} takeoff failed')
                    return

            max_len = max(len(wps) for wps in drones_waypoints.values())
            for i in range(max_len):
                for ns, drone in interfaces.items():
                    wps = drones_waypoints[ns]
                    if i >= len(wps):
                        continue
                    lat, lon = wps[i]
                    drone.go_to.go_to_gps_point([lat, lon, drone_altitudes.get(ns, altitude)], speed=speed)
                self._publish_status('running', f'waypoint {i + 1}/{max_len}', {'waypoint_index': i})

            self._publish_status('returning', 'returning to launch site')
            self._return_to_launch(interfaces, self._launch_local, altitude, speed)

            self._publish_status('landing', 'landing all drones')
            for ns, drone in interfaces.items():
                self._call_physical_action(
                    drone, lambda d=drone: d.land(speed=0.4),
                    (self.STATE_LANDED, self.STATE_DISARMED), f'{ns} land')

            self._publish_status('complete', 'mission finished')
            succeeded = True
        except Exception as e:  # noqa: BLE001 - surface any failure to the GCS instead of dying silently
            self._publish_status('error', f'mission crashed: {e}')
        finally:
            if not succeeded:
                self._disarm_stuck_drones(interfaces)
            self._executing = False

    def _run_boundary_coverage_mission(self, mission: dict, boundary: list):
        """Phase 1 flow: auto zone-split the boundary, generate a lawnmower
        path per zone, publish both for the GCS map, then fly all drones'
        paths concurrently via FollowPath."""
        # float(...): see _run_json_waypoint_mission's comment -- a
        # whole-number speed_mps/altitude_m from the GCS JSON parses as a
        # Python int, and AS2's FollowPath.Goal.max_speed setter asserts
        # strict float typing and raises on an int (confirmed live: crashed
        # every _fly thread below with "must be of type 'float'").
        speed = float(mission.get('speed_mps', 0.5))
        # NOT the JSON-waypoint path's 1.0m default -- that would silently
        # conflict with path_planner's swath-width math, which assumes a
        # realistic scan altitude.
        scan_altitude = float(mission.get('altitude_m', path_planner.DEFAULT_SCAN_ALTITUDE_M))
        # Per-drone altitude override from the GCS (namespace -> meters),
        # falling back to scan_altitude for any drone not listed. Each
        # drone's own zone is planned at its own altitude, since swath width
        # (and therefore line spacing) depends on scan altitude.
        drone_altitudes = {ns: float(a) for ns, a in (mission.get('drone_altitudes') or {}).items()}
        namespaces = self.known_namespaces

        self._publish_status('starting', f'splitting boundary into {len(namespaces)} zones',
                              {'drones': namespaces})
        # Check if mission specifies a custom home launch site
        home = mission.get('home')
        origin_lat = home[0] if (home and isinstance(home, (list, tuple)) and len(home) >= 2) else self.origin[0]
        origin_lon = home[1] if (home and isinstance(home, (list, tuple)) and len(home) >= 2) else self.origin[1]
        origin_alt = self.origin[2]

        # Interfaces created here (not later, at arm time) and reused for the
        # rest of this mission -- _get_interface is idempotent -- so each
        # drone's own real launch position is known before zones are handed
        # out. Without this, zones got paired to namespaces in world-config
        # order, independent of where any drone actually was; drones could
        # be sent to fly the side of the arena nearest a *different* drone.
        # _resolve_drone_positions also applies any GCS-configured
        # drone_launch_positions (teleporting drones within the fixed 12ft
        # launch box) before reading each one's now-current GPS fix.
        interfaces: dict[str, DroneInterfaceGPS] = {ns: self._get_interface(ns) for ns in namespaces}
        drone_positions = self._resolve_drone_positions(mission, interfaces)
        if drone_positions is None:
            self._executing = False
            return

        try:
            zones_latlon = zone_splitter.split_boundary(
                boundary, num_zones=len(namespaces),
                origin_lat=origin_lat, origin_lon=origin_lon, origin_alt=origin_alt)
        except ValueError as e:
            self._publish_status('error', f'zone splitting failed: {e}')
            self._executing = False
            return

        drone_zone = zone_splitter.assign_nearest_zones(drone_positions, zones_latlon)
        drone_waypoints = {}
        for ns, zone in drone_zone.items():
            try:
                drone_waypoints[ns] = path_planner.generate_lawnmower_path(
                    zone, origin_lat=origin_lat, origin_lon=origin_lon, origin_alt=origin_alt,
                    scan_altitude_m=drone_altitudes.get(ns, scan_altitude),
                    # Start each drone's serpentine at the zone corner nearest
                    # its own launch position, so it begins scanning on arrival
                    # instead of transiting the length of its zone first.
                    start_near_latlon=drone_positions.get(ns))
            except ValueError as e:
                self._publish_status('error', f'path planning failed for {ns}: {e}')
                self._executing = False
                return

        # Coverage plan summary -- the single place to look when debugging
        # "did it map the right area / start at the right corner", especially
        # with Gazebo headless where there is nothing to watch. Everything
        # here is derived, not guessed: areas from the actual zone polygons,
        # swath/spacing from path_planner's own camera model.
        self._log_coverage_plan(boundary, drone_zone, drone_waypoints, drone_positions,
                                 origin_lat, origin_lon, origin_alt,
                                 scan_altitude, drone_altitudes)

        # Published before arm/takeoff so the GCS shows zones + planned
        # lawnmower paths immediately on Start, independent of flight progress.
        self._publish_zone_allocations(drone_zone)
        self._publish_planned_paths(drone_waypoints)

        succeeded = False
        try:
            for ns, drone in interfaces.items():
                if not self._call_bounded(drone.arm) or not self._call_bounded(drone.offboard):
                    self._publish_status('error', f'{ns} failed to arm/offboard')
                    return
                if not self._wait_armed_offboard(drone):
                    self._publish_status('error', f'{ns} armed+offboard state never confirmed')
                    return

            self._publish_status('taking_off', 'all drones taking off')
            for ns, drone in interfaces.items():
                ns_altitude = drone_altitudes.get(ns, scan_altitude)
                if not self._call_physical_action(
                        drone, lambda d=drone, a=ns_altitude: d.takeoff(height=a, speed=speed),
                        (self.STATE_FLYING,), f'{ns} takeoff'):
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
                try:
                    results[ns] = drone.follow_path(
                        self._compensate_gps_path(
                            drone, ns, wps, (origin_lat, origin_lon, origin_alt)),
                        speed=speed)
                except Exception as e:  # noqa: BLE001 - a bare thread target's exception is
                    # otherwise printed to stderr and silently swallowed, never reaching this
                    # method's own try/except (that only wraps the main thread). Confirmed
                    # live: an AssertionError here (int vs float mission speed) left `results`
                    # completely empty, the old `failed = [... for ns, ok in results.items()]`
                    # iterated zero items, `if failed:` was falsy, and the mission reported
                    # "complete" -- landing every drone -- without any of them having flown.
                    self.get_logger().error(f'{ns} follow_path crashed: {e}')
                    results[ns] = False

            threads = [threading.Thread(target=_fly, args=(ns, interfaces[ns], wps))
                       for ns, wps in drone_waypoints.items()]
            for t in threads:
                t.start()
            for t in threads:
                t.join()

            # .get(ns), not results.items(): a drone missing from `results`
            # entirely (any future bug that returns before the assignment
            # above) must still count as failed, not silently drop out of
            # this check the way it did before.
            failed = [ns for ns in drone_waypoints if not results.get(ns)]
            if failed:
                self._publish_status('error', f'follow_path failed for: {failed}')
                return

            self._publish_status('returning', 'returning to launch site')
            self._return_to_launch(interfaces, self._launch_local, scan_altitude, speed)

            self._publish_status('landing', 'landing all drones')
            for ns, drone in interfaces.items():
                self._call_physical_action(
                    drone, lambda d=drone: d.land(speed=0.4),
                    (self.STATE_LANDED, self.STATE_DISARMED), f'{ns} land')

            self._publish_status('complete', 'mission finished')
            succeeded = True
        except Exception as e:  # noqa: BLE001 - surface any failure to the GCS instead of dying silently
            self._publish_status('error', f'mission crashed: {e}')
        finally:
            if not succeeded:
                self._disarm_stuck_drones(interfaces)
            self._executing = False


    @staticmethod
    def _polygon_area_m2(points_enu) -> float:
        """Shoelace area of an ENU-metre polygon."""
        n = len(points_enu)
        if n < 3:
            return 0.0
        return abs(sum(points_enu[i][0] * points_enu[(i + 1) % n][1]
                       - points_enu[(i + 1) % n][0] * points_enu[i][1]
                       for i in range(n))) / 2.0

    def _log_coverage_plan(self, boundary, drone_zone, drone_waypoints, drone_positions,
                            origin_lat, origin_lon, origin_alt,
                            scan_altitude, drone_altitudes):
        """Log the full derived coverage plan: mapping area, per-zone size,
        camera swath/line spacing, line count, path length, and which corner
        each drone starts from plus its transit distance."""
        try:
            b_enu = geo_utils.latlon_to_enu(boundary, origin_lat, origin_lon, origin_alt)
            total_area = self._polygon_area_m2(b_enu)
            fp_long, fp_short = path_planner.ground_footprint_m(scan_altitude)
            swath = path_planner.swath_width_m(scan_altitude)
            spacing = swath * (1.0 - path_planner.DEFAULT_OVERLAP_PCT / 100.0)

            self.get_logger().info(
                f'[coverage] mapping area {total_area:.0f} m^2 | {len(drone_zone)} zones | '
                f'alt {scan_altitude:.1f}m | camera hfov '
                f'{path_planner.DEFAULT_CAMERA_HFOV_DEG:.0f}deg vfov '
                f'{path_planner.camera_vfov_deg():.1f}deg | frame {fp_long:.2f}x{fp_short:.2f}m '
                f'| swath {swath:.2f}m (short edge) | line spacing {spacing:.2f}m '
                f'@ {path_planner.DEFAULT_OVERLAP_PCT:.0f}% overlap')

            for ns, zone in drone_zone.items():
                z_enu = geo_utils.latlon_to_enu(zone, origin_lat, origin_lon, origin_alt)
                xs = [p[0] for p in z_enu]
                ys = [p[1] for p in z_enu]
                wps = drone_waypoints.get(ns, [])
                w_enu = geo_utils.latlon_to_enu([(w[0], w[1]) for w in wps],
                                                 origin_lat, origin_lon, origin_alt)
                path_len = sum(math.dist(w_enu[i], w_enu[i + 1]) for i in range(len(w_enu) - 1))
                start = drone_positions.get(ns)
                transit = float('nan')
                corner = '?'
                if start and w_enu:
                    (sx, sy), = geo_utils.latlon_to_enu([start], origin_lat, origin_lon, origin_alt)
                    transit = math.dist((sx, sy), w_enu[0])
                    corner = ('S' if w_enu[0][1] - min(ys) < max(ys) - w_enu[0][1] else 'N') + \
                             ('W' if w_enu[0][0] - min(xs) < max(xs) - w_enu[0][0] else 'E')
                self.get_logger().info(
                    f'[coverage] {ns}: zone {max(xs) - min(xs):.1f}x{max(ys) - min(ys):.1f}m '
                    f'= {self._polygon_area_m2(z_enu):.0f} m^2 | alt '
                    f'{drone_altitudes.get(ns, scan_altitude):.1f}m | {len(wps) // 2} lines, '
                    f'{len(wps)} waypoints | path {path_len:.0f}m | starts {corner} corner, '
                    f'transit {transit:.1f}m')
        except Exception as e:  # noqa: BLE001 - diagnostics must never abort a mission
            self.get_logger().warn(f'[coverage] could not log plan summary: {e}')

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

    def _on_drone_command(self, msg: DroneCommand):
        threading.Thread(target=self._execute_drone_command, args=(msg,), daemon=True).start()

    def _execute_drone_command(self, cmd: DroneCommand):
        action = cmd.action
        target = cmd.drone_id
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

    def _execute_drone_command_one(self, ns: str, action: str, cmd: DroneCommand):
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
                if ok:
                    ok = self._wait_armed_offboard(drone)
            elif action == 'set_launch_position':
                # Interactive placement from the GCS: move the drone NOW so the
                # operator sees it happen in Gazebo, instead of only applying
                # the position when the mission starts. Mission start still
                # re-applies drone_launch_positions, so the two paths agree.
                if not (math.isfinite(cmd.latitude) and math.isfinite(cmd.longitude)):
                    self._publish_control_status(ns, action, False, 'missing lat/lon')
                    return
                if self._executing:
                    self._publish_control_status(ns, action, False, 'mission in progress')
                    return
                if drone.info.get('state') not in (self.STATE_DISARMED, self.STATE_LANDED):
                    self._publish_control_status(
                        ns, action, False, 'drone must be landed/disarmed to reposition')
                    return
                box_center = self._launch_box_center
                if box_center is not None and not self._inside_launch_box(
                        cmd.latitude, cmd.longitude, box_center):
                    self._publish_control_status(
                        ns, action, False,
                        f'outside the {LAUNCH_BOX_SIZE_M:.2f}m '
                        f'({LAUNCH_BOX_SIZE_M / 0.3048:.0f}ft) launch box')
                    return
                ok = self._teleport_drone(ns, cmd.latitude, cmd.longitude)
            elif action == 'disarm':
                ok = self._call_bounded(drone.disarm)
            elif action == 'takeoff':
                # 0.0 is the bridge's "not specified" sentinel -- see
                # nidar_gcs_bridge's module docstring.
                altitude = cmd.altitude_m if cmd.altitude_m > 0.0 else path_planner.DEFAULT_SCAN_ALTITUDE_M
                # Cheap even when already armed+offboard from an earlier,
                # separate Arm click -- first poll returns True immediately.
                # Guards the same race as the mission flows: a takeoff
                # requested right after Arm can otherwise hit TakeoffBehavior
                # before its own FSM has processed the state change.
                if not self._wait_armed_offboard(drone):
                    self._publish_control_status(ns, action, False, 'drone not confirmed armed+offboard')
                    return
                ok = self._call_physical_action(
                    drone, lambda d=drone: d.takeoff(height=altitude, speed=0.5),
                    (self.STATE_FLYING,), f'{ns} takeoff')
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


def main():
    rclpy.init()
    node = MissionExecutor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
