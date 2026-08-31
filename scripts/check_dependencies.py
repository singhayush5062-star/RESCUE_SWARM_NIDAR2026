#!/usr/bin/env python3
"""Fail fast, and by name, when a Python package the runtime needs is missing.

Written after a silent outage: `filelock` vanished from the user site-packages
(it arrives as a torch dependency, so nothing asked for it by name), and
ultralytics imports it deep inside a lazy import chain. All four detection
nodes died at startup, and because the GCS video feed is published BY those
nodes, the operator lost the camera view and every detection at the same time.
The launcher still printed "simulation is up". The only evidence was a
traceback at the bottom of /tmp/nidar_sim_logs/detection.log.

Two design choices worth keeping:

* importlib.util.find_spec, not a real import. Importing torch and ultralytics
  costs several seconds each and would slow every launch; find_spec answers
  "is this package present" without executing it, and would have caught the
  filelock outage exactly.
* The interpreter is checked too. conda's python shadowing the system one
  breaks rclpy for every ROS 2 tool, which this project has been bitten by
  before -- setup_nidar_ros.sh strips conda from PATH for that reason. A
  missing package reported against the WRONG interpreter sends you off fixing
  an environment the simulator never uses.

Exit status is 0 only when everything the runtime needs is importable.
"""

import importlib.util
import sys

#: (module to import, package to install, what breaks without it).
#: Transitive dependencies are listed explicitly on purpose. filelock and lap
#: are reached only through ultralytics' own lazy imports, so nothing in this
#: repo mentions them -- which is precisely why losing one was invisible.
PIP_REQUIRED = [
    ('ultralytics', 'ultralytics', 'person detection'),
    ('torch', 'torch', 'person detection'),
    ('torchvision', 'torchvision', 'person detection'),
    ('ncnn', 'ncnn', 'CPU detection backend'),
    ('filelock', 'filelock', 'ultralytics import chain'),
    ('lap', 'lap', 'ByteTrack tracker'),
    ('websocket', 'websocket-client', 'telemetry health check'),
]

#: Provided by apt / ROS in /usr/lib/python3/dist-packages. pip must not be
#: pointed at these -- shadowing an apt build of numpy or cv2 is a good way to
#: break cv_bridge -- so they get different repair advice.
APT_REQUIRED = [
    ('numpy', 'python3-numpy', 'everything'),
    ('cv2', 'python3-opencv', 'image handling'),
    ('yaml', 'python3-yaml', 'world/mission config'),
    ('pymap3d', 'python3-pymap3d', 'GPS <-> ENU conversion'),
]


def _missing(entries):
    out = []
    for module, package, purpose in entries:
        try:
            found = importlib.util.find_spec(module) is not None
        except (ImportError, ValueError):
            found = False
        if not found:
            out.append((module, package, purpose))
    return out


def main():
    if 'conda' in sys.executable or 'miniconda' in sys.executable:
        print(f'  WRONG INTERPRETER: {sys.executable}')
        print('  The simulator runs on the system python3; conda\'s python breaks rclpy.')
        print('  Run this via /usr/bin/python3, or source scripts/setup_nidar_ros.sh first.')
        return 1

    pip_missing = _missing(PIP_REQUIRED)
    apt_missing = _missing(APT_REQUIRED)
    if not pip_missing and not apt_missing:
        total = len(PIP_REQUIRED) + len(APT_REQUIRED)
        print(f'  all {total} runtime packages present ({sys.executable})')
        return 0

    print(f'  MISSING PYTHON PACKAGES for {sys.executable}')
    for module, package, purpose in pip_missing + apt_missing:
        print(f'    {module:<14} ({package}) -- needed for {purpose}')
    print()
    print('  Detection nodes will die at startup, which also blanks the GCS')
    print('  video feed: the video is published by those same nodes.')
    print()
    if pip_missing:
        print('  Fix:  /usr/bin/python3 -m pip install --user -r requirements.txt')
    if apt_missing:
        print('  Fix:  sudo apt install ' + ' '.join(p for _, p, _ in apt_missing))
    return 1


if __name__ == '__main__':
    sys.exit(main())
