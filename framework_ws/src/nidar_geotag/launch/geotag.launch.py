"""Launches one geotag node per drone plus the single swarm aggregator.

Mirrors nidar_detection/launch/detection.launch.py: one launch process for
the whole swarm rather than one per drone.

Usage:
    ros2 launch nidar_geotag geotag.launch.py \\
        drone_ids:=drone0,drone1,drone2,drone3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn(context):
    drone_ids = [d.strip() for d in
                 LaunchConfiguration('drone_ids').perform(context).split(',') if d.strip()]
    min_conf = float(LaunchConfiguration('min_confidence').perform(context))
    ground_alt = float(LaunchConfiguration('ground_altitude_m').perform(context))
    local_r = float(LaunchConfiguration('local_dedup_radius_m').perform(context))
    global_r = float(LaunchConfiguration('global_dedup_radius_m').perform(context))
    # Must match the clock the TF publishers use. Gazebo and every AS2 node run
    # on sim time, and a detection carries the camera frame's own sim-time
    # stamp; a geotag node left on the wall clock sees those transforms as
    # decades stale and evicts them from the tf2 cache immediately, so every
    # single lookup fails. Measured with this unset: 3 of 4 drones dropped
    # 100% of detections as no_tf and produced zero tags. Defaults to false so
    # real hardware, which has no /clock, is unaffected.
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() \
        in ('true', '1', 'yes')

    nodes = [
        Node(
            package='nidar_geotag',
            executable='geotag_node',
            # Named per drone so four identical nodes stay distinguishable in
            # ros2 node list and, more importantly, in /rosout -- the GCS log
            # console filters by source name.
            name=f'geotag_{ns}',
            output='screen',
            parameters=[{
                'drone_id': ns,
                'min_confidence': min_conf,
                'ground_altitude_m': ground_alt,
                'local_dedup_radius_m': local_r,
                'use_sim_time': use_sim_time,
            }],
        )
        for ns in drone_ids
    ]
    nodes.append(Node(
        package='nidar_geotag',
        executable='survivor_aggregator_node',
        name='survivor_aggregator',
        output='screen',
        parameters=[{
            'drone_ids': drone_ids,
            'global_dedup_radius_m': global_r,
            'use_sim_time': use_sim_time,
        }],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_ids', default_value='drone0,drone1,drone2,drone3'),
        DeclareLaunchArgument('min_confidence', default_value='0.5'),
        DeclareLaunchArgument('ground_altitude_m', default_value='0.0'),
        DeclareLaunchArgument('local_dedup_radius_m', default_value='2.0'),
        DeclareLaunchArgument('global_dedup_radius_m', default_value='2.5'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_spawn),
    ])
