# Project Gazebo

Please refer to https://aerostack2.github.io/_02_examples/gazebo/project_gazebo/index.html for more information.

## Installation

To install this project, clone the repository:

```bash
git clone https://github.com/aerostack2/project_gazebo
```

To start using this project, please go to the root folder of the project.

## Execution

### 1. Launch aerostack2 nodes for each drone
To launch aerostack2 nodes for each drone, execute once the following command:

```bash
./launch_as2.bash
```

The flags for the components launcher are:

- **-m**: multi agent. Default not set
- **-n**: select drones namespace to launch, values are comma separated. By default, it will get all drones from world description file
- **-s**: if set, the simulation will not be launched. Default launch simulation
- **-g**: launch using gnome-terminal instead of tmux. Default not set

### 2. Launch a mission

NIDAR RescueSwarm's actual entry point is the single top-level launcher, not the
per-mission demo scripts this template originally shipped with (removed — superseded
entirely by the GCS-driven flow below):

```bash
./scripts/run_simulation.sh
```

This brings up Gazebo + AS2 for every drone in `config/world_swarm.yaml`, rosbridge,
the `nidar_gcs_bridge`/`nidar_mission_executor`/`nidar_survivor_manager` ROS 2 nodes
(see `DOCUMENTS/standard_implementation_plan_ros2_framework.md`), and the GCS dev
server. Missions are loaded and started from the GCS web UI, not from the CLI.

### 3. End the execution

If you are using tmux, you can end the execution with the following command:

```bash
./stop.bash
```

You can force the end of all tmux sessions with the command:
```bash
tmux kill-server
```

If you are using gnome-terminal, you can end the execution by closing the terminal.


## Developers guide

All projects in aerostack2 are structured in the same way. The project is divided into the following directories:

- **tmuxinator**: Contains the tmuxinator launch file, which is used to launch all aerostack2 nodes.
  - **aerostack2.yaml**: Tmuxinator launch file for each drone. The list of nodes to be launched is defined here.
- **config**: Contains the configuration files for the launchers of the nodes in the drones.
- **config_ground_station**: Contains the configuration files for the launchers of the nodes in the ground station.
- **launch_as2.bash**: Script to launch nodes defined in *tmuxinator/aerostack2.yaml* (called by `scripts/run_simulation.sh`, not run directly).
- **stop_tmuxinator_as2.bash**: Script to stop all nodes launched by *launch_as2.bash*.
- **stop_tmuxinator_ground_station.bash**: Script to stop all nodes launched by *launch_ground_station.bash*.
- **stop_tmuxinator.bash**: Script to stop all nodes launched by *launch_as2.bash* and *launch_ground_station.bash*.
- **rosbag/record_rosbag.bash**: Script to record a rosbag. Can be modified to record only the topics that are needed.
- **trees\***: Contains the behavior trees that can be executed. They can be selected in the *aerostack2.yaml* file.
- **utils**: Contains utils scripts for launchers.

Both python and bash scripts have a help message that can be displayed by running the script with the `-h` option. For example, `./launch_as2.bash -h` will display the help message for the `launch_as2.bash` script.

**Note**: For knowing all parameters for each launch, you can execute the following command:

```bash
ros2 launch my_package my_launch.py -s
```

Also, you can see them in the default config file of the package, in the *config* folder. If you want to modify the default parameters, you can add the parameter to the config file.