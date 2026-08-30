#!/usr/bin/env python3
"""Verify every drone's telemetry actually reaches the GCS, via rosbridge.

Exists because the stack's worst failure mode is coming up *partially*
connected while reporting success. ROS 2 discovery on this host runs almost
entirely over FastDDS shared memory (there is no 224.0.0.0/4 multicast route,
so the UDP path never carries it). Past roughly a hundred participants,
FastDDS starts failing to acquire shared-memory ports:

  [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7709:
  open_and_lock_file failed

A participant that hits this maps only part of the graph, permanently -- the
subset is fixed when it starts. When the participant is rosbridge, whichever
drones fall in the gap never deliver a NavSatFix to the browser and read
OFFLINE, while flying perfectly and publishing normally to every other node.
Confirmed live: drone0/drone1 delivered 731 and 722 GPS messages in 25s while
drone2/drone3 delivered none, purely because rosbridge could not see their
gps_bridge.

Checking from a fresh ROS 2 participant would NOT catch this -- that probe
gets its own arbitrary subset. The only authoritative view of what the
operator sees is rosbridge's, so this asks rosbridge directly.

Exit status is 0 only when every drone delivered telemetry.
"""

import json
import sys
import time

import websocket

TOPIC = 'sensor_measurements/gps'
MSG_TYPE = 'sensor_msgs/msg/NavSatFix'


def check(drones, url, window_sec):
    try:
        ws = websocket.create_connection(url, timeout=window_sec + 10)
    except Exception as exc:  # noqa: BLE001 - any failure here means no GCS data at all
        print(f'  cannot reach rosbridge at {url}: {exc}')
        return {d: 0 for d in drones}

    for drone in drones:
        ws.send(json.dumps({'op': 'subscribe', 'topic': f'/{drone}/{TOPIC}',
                            'type': MSG_TYPE, 'id': drone}))

    counts = {d: 0 for d in drones}
    ws.settimeout(2)
    deadline = time.monotonic() + window_sec
    while time.monotonic() < deadline and not all(counts.values()):
        try:
            msg = json.loads(ws.recv())
        except Exception:  # noqa: BLE001 - a quiet gap is normal, keep waiting
            continue
        if msg.get('op') == 'publish':
            drone = msg['topic'].split('/')[1]
            if drone in counts:
                counts[drone] += 1
    ws.close()
    return counts


def main():
    drones = [d for d in sys.argv[1].split(',') if d] if len(sys.argv) > 1 else []
    window_sec = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    url = sys.argv[3] if len(sys.argv) > 3 else 'ws://127.0.0.1:9090'
    if not drones:
        print('usage: check_telemetry.py drone0,drone1 [seconds] [ws_url]')
        return 2

    counts = check(drones, url, window_sec)
    silent = [d for d in drones if not counts[d]]

    for drone in drones:
        n = counts[drone]
        print(f'  {drone}: {n:4d} GPS msgs  {"ok" if n else "SILENT -- will show OFFLINE in the GCS"}')

    if silent:
        print()
        print(f'FAILED: {len(silent)} of {len(drones)} drones deliver no telemetry to the GCS: '
              f'{", ".join(silent)}')
        print('These drones are almost certainly flying fine -- this is a DDS discovery')
        print('failure at rosbridge, not a flight failure. Check, in order:')
        print('  1. a multicast route exists:  ip route show | grep 224.0.0.0/4')
        print('  2. participant count:         ps -eo args | grep -c parameter_bridge')
        print('  3. rosbridge SHM errors:      grep "Failed init_port" /tmp/nidar_sim_logs/rosbridge.log')
        return 1

    print(f'  all {len(drones)} drones delivering telemetry to the GCS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
