"""Merges every drone's survivor tags into one swarm-wide list.

One node for the whole swarm (unlike geotag_node, which is per-drone),
because its entire job is the thing no single drone can do: recognise that
drone1's survivor 3 and drone2's survivor 0 are the same person.

    /<drone>/geotag/survivors  x N  (SurvivorTag)
                |
                |  geographic dedup at global_dedup_radius_m
                v
    /nidar/survivors/aggregated     (SurvivorList, full snapshot)

Why geography and not track ids: ByteTrack ids are scoped to one drone's
camera (see DetectionResult.msg), so two drones looking at one person produce
two unrelated ids. Position is the only thing they agree on.

The per-drone tags arriving here are already deduplicated *within* each drone
and carry that drone's running mean position, so this node is merging a
handful of stable estimates, not a raw detection firehose.
"""

from typing import Dict

import rclpy
from rclpy.node import Node

from nidar_msgs.msg import SurvivorList, SurvivorTag
from nidar_mission_manager import world_config
from nidar_mission_manager.survivor_aggregator import (
    DEFAULT_GLOBAL_DEDUP_RADIUS_M, SurvivorRegistry)

from pathlib import Path

WORLD_CONFIG_PATH = Path('config/world_swarm.yaml')


class SurvivorAggregatorNode(Node):

    def __init__(self):
        super().__init__('survivor_aggregator')

        self.declare_parameter('drone_ids', [''])
        self.declare_parameter('global_dedup_radius_m', DEFAULT_GLOBAL_DEDUP_RADIUS_M)
        self.declare_parameter('publish_rate_hz', 2.0)

        drone_ids = [d for d in self.get_parameter('drone_ids').value if d]
        if not drone_ids:
            drone_ids = world_config.get_drones_namespaces(WORLD_CONFIG_PATH)
        self.drone_ids = drone_ids

        self.registry = SurvivorRegistry(
            dedup_radius_m=float(self.get_parameter('global_dedup_radius_m').value))

        self.list_pub = self.create_publisher(
            SurvivorList, '/nidar/survivors/aggregated', 10)

        for ns in self.drone_ids:
            self.create_subscription(
                SurvivorTag, f'/{ns}/geotag/survivors', self._on_tag, 10)

        # Republished on a timer rather than on every tag. A drone re-observes
        # one survivor many times a second; forwarding each one would push the
        # same near-identical list to the GCS at detection rate, for a marker
        # that moves centimetres. The timer alone solves that -- there is
        # deliberately NO "only publish when something changed" gate.
        #
        # That gate was tried and was wrong. Survivor positions stop changing
        # the moment the mission ends, so the last snapshot went out during
        # the flight and nothing was ever published again. Any consumer that
        # connected afterwards -- an operator opening the GCS to review the
        # run, which is the normal way results get looked at -- saw an empty
        # map with every survivor still sitting in the aggregator. Measured
        # exactly that: 28 per-drone survivor records held, 0 delivered.
        #
        # A steady snapshot of a single-digit list at 2 Hz costs nothing and
        # makes every consumer correct whenever it happens to connect,
        # without depending on QoS durability surviving the relay through
        # nidar_gcs_bridge and rosbridge to the browser.
        rate = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._publish_list)

        self._per_drone: Dict[str, int] = {ns: 0 for ns in self.drone_ids}

        self.get_logger().info(
            f'survivor_aggregator ready | drones={",".join(self.drone_ids)} '
            f'| dedup={self.registry.dedup_radius_m:.1f}m | {rate:g} Hz')

    def _on_tag(self, tag: SurvivorTag):
        ns = tag.detecting_drone_id
        self._per_drone[ns] = self._per_drone.get(ns, 0) + 1
        # The per-drone survivor_id is a stable identity within that drone, so
        # it is a definitive merge key here too -- a drone that already
        # decided two sightings are one person must not have them split again.
        _, is_new = self.registry.observe(
            tag.latitude, tag.longitude, tag.altitude,
            tag.confidence, ns, key=f'{ns}:{tag.survivor_id}')
        if is_new:
            self.get_logger().info(
                f'survivor {len(self.registry) - 1} added from {ns} '
                f'({tag.latitude:.7f},{tag.longitude:.7f}) conf {tag.confidence:.2f} '
                f'| {len(self.registry)} unique survivor(s) known')

    def _publish_list(self):
        # Publish even with nothing found yet: an empty list is a real,
        # useful statement ("the swarm has found no one"), and it is what
        # tells a freshly-connected GCS that this topic is alive at all.
        msg = SurvivorList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'earth'
        for r in self.registry.records:
            tag = SurvivorTag()
            tag.header = msg.header
            tag.survivor_id = r.survivor_id
            tag.latitude = r.latitude
            tag.longitude = r.longitude
            tag.altitude = r.altitude
            tag.confidence = r.confidence
            # Which drone gets the credit: the first to see them. The full
            # set is in the log; SurvivorTag has one field for this.
            tag.detecting_drone_id = (r.detecting_drones[0]
                                      if r.detecting_drones else '')
            tag.delivery_assigned = False
            tag.delivery_complete = False
            msg.survivors.append(tag)
        self.list_pub.publish(msg)


def main():
    rclpy.init()
    node = SurvivorAggregatorNode()
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
