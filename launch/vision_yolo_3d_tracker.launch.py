"""Launch the vision_yolo_3d_tracker pipeline."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('vision_yolo_3d_tracker')

    detector_config = os.path.join(pkg_share, 'config', 'detector_params.yaml')
    tracker_config = os.path.join(pkg_share, 'config', 'tracker_params.yaml')
    viz_config = os.path.join(pkg_share, 'config', 'viz_params.yaml')
    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'tracking_visualization.rviz',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    run_rviz = LaunchConfiguration('run_rviz')

    detector = Node(
        package='vision_yolo_3d_tracker',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_path': detector_config},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    tracker = Node(
        package='vision_yolo_3d_tracker',
        executable='kalman_tracker_node',
        name='kalman_tracker_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_path': tracker_config},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    visualization = Node(
        package='vision_yolo_3d_tracker',
        executable='visualization_node',
        name='visualization_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_path': viz_config},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='vision_yolo_3d_tracker_rviz',
        output='log',
        arguments=['-d', rviz_config],
        condition=IfCondition(run_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulated time',
            ),
            DeclareLaunchArgument(
                'log_level',
                default_value='info',
                description='Logging level (debug, info, warn, error)',
            ),
            DeclareLaunchArgument(
                'run_rviz',
                default_value='true',
                description='Start RViz2 with the packaged config',
            ),
            detector,
            tracker,
            visualization,
            rviz2,
        ]
    )
