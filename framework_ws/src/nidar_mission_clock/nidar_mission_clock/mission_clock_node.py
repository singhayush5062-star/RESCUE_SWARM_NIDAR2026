#!/usr/bin/env python3
"""Owns the mission clock: how long the current (or last) mission run took.

One node, one task. This node does not fly, plan, or command anything -- it
observes the mission state stream nidar_mission_executor already publishes
and turns it into a typed nidar_msgs/MissionStatus carrying elapsed time,
phase, and active drone count.

Why a separate node rather than a timestamp inside the executor: the
executor spends most of a mission blocked inside AS2 calls (follow_path
joins every drone's thread, which for a real coverage run is minutes with
no return to its own event loop). It therefore *cannot* publish a ticking
clock -- its status messages only appear at phase transitions, which can be
many minutes apart. A node with its own timer can, so the GCS gets a
steadily-advancing server-side clock instead of interpolating between
sparse events and hoping it stayed in sync.

Not filled in here: total_survivors_detected / total_deliveries_complete.
Those belong to the detection and delivery pipelines (Phase 2/3/4), not to
a clock, and are left at 0 rather than guessed. When the survivor
aggregator exists it publishes its own counts and this node folds them in
via _on_survivor_count -- one subscription, no change to who owns what.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from nidar_msgs.msg import MissionStatus

from nidar_mission_clock.clock import MissionClock, format_duration

#: How often the elapsed time is republished. 1 Hz is deliberately slow:
#: the display is whole seconds, and this topic crosses rosbridge into a
#: browser that has already been driven out of memory once this project by
#: unthrottled high-rate topics.
PUBLISH_PERIOD_SEC = 1.0

#: Executor state name -> MissionStatus phase enum. Unknown states fall back
#: to SETUP rather than raising, so adding a state to the executor can never
#: take this node down.
PHASE_CODES = {
    'idle': MissionStatus.SETUP,
    'loaded': MissionStatus.SETUP,
    'starting': MissionStatus.SETUP,
    'taking_off': MissionStatus.SETUP,
    'running': MissionStatus.SCANNING,
    'returning': MissionStatus.RTL,
    'landing': MissionStatus.RTL,
    'complete': MissionStatus.COMPLETE,
    'error': MissionStatus.ABORTED,
}


class MissionClockNode(Node):

    def __init__(self):
        super().__init__('mission_clock')

        self._mission_clock = MissionClock()
        self._active_drones = 0
        self._survivors_detected = 0
        self._deliveries_complete = 0

        self.progress_pub = self.create_publisher(MissionStatus, '/nidar/mission_progress', 10)
        self.create_subscription(String, '/nidar/mission_status', self._on_mission_status, 10)
        # Published by the Phase 3 survivor aggregator once it exists.
        # Subscribing now costs nothing (a subscription to an unpublished
        # topic is simply idle) and means the aggregator can be added later
        # without touching this node.
        self.create_subscription(String, '/nidar/survivor_count', self._on_survivor_count, 10)

        self.create_timer(PUBLISH_PERIOD_SEC, self._publish_progress)

        self.get_logger().info('mission_clock ready')

    # ------------------------------------------------------------------

    def _on_mission_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            # The executor is the only publisher and always sends valid
            # JSON; a malformed message means something else is on the
            # topic, which is not this node's problem to fix or die over.
            return

        state = str(payload.get('state', ''))
        if not state:
            return

        drones = payload.get('drones')
        if isinstance(drones, list):
            self._active_drones = len(drones)

        was_running = self._mission_clock.running
        if not was_running and state not in ('idle', 'loaded', 'complete', 'error'):
            # A new run is about to start -- clear the previous run's tallies
            # so they can't be mistaken for this one's.
            self._survivors_detected = 0
            self._deliveries_complete = 0

        self._mission_clock.on_state(state)

        if was_running and not self._mission_clock.running:
            self._log_breakdown(state)
        elif not was_running and self._mission_clock.running:
            self.get_logger().info(f'[clock] mission started (phase: {state})')

        # Publish immediately on a transition rather than waiting up to a
        # full second, so the GCS phase label and the mission status it sits
        # next to never disagree.
        self._publish_progress()

    def _on_survivor_count(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        detected = payload.get('detected')
        delivered = payload.get('delivered')
        if isinstance(detected, int):
            self._survivors_detected = detected
        if isinstance(delivered, int):
            self._deliveries_complete = delivered

    # ------------------------------------------------------------------

    def _publish_progress(self):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.phase = PHASE_CODES.get(self._mission_clock.state, MissionStatus.SETUP)
        msg.elapsed_time_sec = float(self._mission_clock.elapsed())
        msg.mission_running = self._mission_clock.running
        msg.active_drones = min(255, self._active_drones)
        msg.total_survivors_detected = min(255, self._survivors_detected)
        msg.total_deliveries_complete = min(255, self._deliveries_complete)
        self.progress_pub.publish(msg)

    def _log_breakdown(self, final_state: str):
        """Log where a finished run's time actually went.

        This is the counterpart to the executor's own [coverage] plan log:
        that one says what the mission intended to do, this one says how
        long each part of doing it took. Both land in the same place, so
        `grep '\\[clock\\]'` answers "why did that run take so long" without
        a running simulator to watch.
        """
        total = self._mission_clock.elapsed()
        verb = 'completed' if final_state == 'complete' else f'ended ({final_state})'
        self.get_logger().info(f'[clock] mission {verb} in {format_duration(total)} ({total:.1f}s)')
        for name, seconds in self._mission_clock.phase_durations():
            share = (seconds / total * 100.0) if total > 0 else 0.0
            self.get_logger().info(
                f'[clock]   {name:<12} {format_duration(seconds):>10}  ({share:4.1f}%)')


def main():
    rclpy.init()
    node = MissionClockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
