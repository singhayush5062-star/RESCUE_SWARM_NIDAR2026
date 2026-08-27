import json

import pytest

from nidar_mission_manager import world_config

WORLD_YAML = """\
world_name: "grass"
origin:
    latitude: 28.682412
    longitude: 77.499734
    altitude: 100.0
drones:
  - model_type: "px4vision"
    model_name: "drone0"
  - model_type: "px4vision"
    model_name: "drone1"
"""


@pytest.fixture
def yaml_config(tmp_path):
    p = tmp_path / 'world_swarm.yaml'
    p.write_text(WORLD_YAML)
    return p


def test_load_origin(yaml_config):
    assert world_config.load_origin(yaml_config) == (28.682412, 77.499734, 100.0)


def test_load_world_name(yaml_config):
    assert world_config.load_world_name(yaml_config) == 'grass'


def test_get_drones_namespaces_gazebo_shape(yaml_config):
    assert world_config.get_drones_namespaces(yaml_config) == ['drone0', 'drone1']


def test_get_drones_namespaces_px4_sitl_shape(tmp_path):
    p = tmp_path / 'sitl.yaml'
    p.write_text('drones:\n  - namespace: "drone0"\n  - namespace: "drone1"\n')
    assert world_config.get_drones_namespaces(p) == ['drone0', 'drone1']


def test_get_drones_namespaces_multirotor_sim_shape(tmp_path):
    p = tmp_path / 'sim.yaml'
    p.write_text('drone0:\n  foo: bar\ndrone1:\n  foo: baz\n')
    assert world_config.get_drones_namespaces(p) == ['drone0', 'drone1']


def test_get_drones_namespaces_json(tmp_path):
    p = tmp_path / 'world_swarm.json'
    p.write_text(json.dumps({'drones': [{'model_name': 'drone0'}]}))
    assert world_config.get_drones_namespaces(p) == ['drone0']


def test_get_drones_namespaces_empty_raises(tmp_path):
    p = tmp_path / 'empty.yaml'
    p.write_text('drones: []\n')
    with pytest.raises(ValueError):
        world_config.get_drones_namespaces(p)
