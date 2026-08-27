"""Shared world/drone-config parsing, zero ROS dependencies.

Used by nidar_mission_executor and nidar_survivor_manager so both nodes read
project_gazebo/config/world_swarm.yaml the same way, and by
project_gazebo/utils/get_drones.py's CLI shim.
"""

import json

import yaml


def _read_config(path) -> dict:
    path = str(path)
    if path.endswith('.json'):
        with open(path, 'r', encoding='utf-8') as stream:
            return json.load(stream)
    if path.endswith('.yaml') or path.endswith('.yml'):
        with open(path, 'r', encoding='utf-8') as stream:
            return yaml.safe_load(stream)
    raise ValueError('Invalid configuration file extension.')


def load_origin(path) -> tuple[float, float, float]:
    """Return (latitude, longitude, altitude) from the config's 'origin' key."""
    cfg = _read_config(path)
    o = cfg['origin']
    return o['latitude'], o['longitude'], o['altitude']


def load_world_name(path) -> str:
    """Return the config's 'world_name' key."""
    cfg = _read_config(path)
    return cfg['world_name']


def get_drones_namespaces(path) -> list[str]:
    """Return drone namespaces listed in the config file (JSON or YAML).

    Supports Gazebo ('model_name'), PX4 SITL ('namespace'), and AS2
    Multirotor Simulator (bare top-level keys) config shapes.
    """
    cfg = _read_config(path)
    namespaces = []

    if 'drones' in cfg:
        for drone in cfg['drones']:
            if 'model_name' in drone:
                namespaces.append(drone['model_name'])
            elif 'namespace' in drone:
                namespaces.append(drone['namespace'])
    else:
        for key in cfg:
            if key == '/**':
                continue
            namespaces.append(key)

    if not namespaces:
        raise ValueError('No drones found in config file')
    return namespaces
