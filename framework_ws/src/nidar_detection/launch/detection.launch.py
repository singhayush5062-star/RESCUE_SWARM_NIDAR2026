"""Launches one detection node per drone.

One `ros2 launch` process covering the whole swarm rather than one per
drone: this project has already had to strip vestigial processes out of its
tmuxinator config once, and four extra launch supervisors buy nothing when
the nodes they start are identical apart from a namespace.

Usage:
    ros2 launch nidar_detection detection.launch.py \\
        drone_ids:=drone0,drone1,drone2,drone3 \\
        model_path:=/path/to/yolov8n.pt
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn(context):
    drone_ids = [d.strip() for d in
                 LaunchConfiguration('drone_ids').perform(context).split(',') if d.strip()]
    model_path = LaunchConfiguration('model_path').perform(context)
    params = {
        'model_path': model_path,
        'confidence_threshold': float(
            LaunchConfiguration('confidence_threshold').perform(context)),
        'inference_rate_hz': float(
            LaunchConfiguration('inference_rate_hz').perform(context)),
        'device': LaunchConfiguration('device').perform(context),
        'input_size': int(LaunchConfiguration('input_size').perform(context)),
        'prefer_ncnn_on_cpu':
            LaunchConfiguration('prefer_ncnn_on_cpu').perform(context).lower()
            in ('1', 'true', 'yes'),
        # Detections carry the camera frame's own stamp downstream to
        # nidar_geotag, which looks up TF at exactly that time. In simulation
        # that stamp is sim time, so this node has to be on the same clock as
        # Gazebo and AS2 or the stamp it forwards is meaningless. Defaults to
        # false so real hardware, which has no /clock, is unaffected.
        'use_sim_time':
            LaunchConfiguration('use_sim_time').perform(context).lower()
            in ('1', 'true', 'yes'),
    }
    return [
        Node(
            package='nidar_detection',
            executable='detection_node',
            # Named per drone rather than left to ROS's default: four nodes
            # sharing the name /detection makes `ros2 node list` useless for
            # telling which drone's detector is missing.
            name=f'detection_{drone_id}',
            output='screen',
            parameters=[dict(params, drone_id=drone_id)],
        )
        for drone_id in drone_ids
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_ids', default_value='drone0,drone1,drone2,drone3',
                              description='Comma-separated drone namespaces.'),
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/ayush/NIDAR/project_gazebo/models/detection/nidar_person.pt',
            description='Path to YOLO weights (.pt / .onnx / .engine, or an '
                        'ncnn export directory). Swapping in retrained weights '
                        'is a copy over this file.'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.5'),
        DeclareLaunchArgument('inference_rate_hz', default_value='2.0'),
        DeclareLaunchArgument('device', default_value='auto',
                              description="'auto', 'cpu', or a CUDA device index."),
        DeclareLaunchArgument('input_size', default_value='640'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'prefer_ncnn_on_cpu', default_value='true',
            description='On CPU, load the *_ncnn_model/ export beside model_path '
                        'when one exists (3x faster, same weights). No effect on GPU.'),
        OpaqueFunction(function=_spawn),
    ])
