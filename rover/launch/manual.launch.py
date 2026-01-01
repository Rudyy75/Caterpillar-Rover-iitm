#!/usr/bin/env python3
"""
Manual Mode Launch File (final)

Provides:
 - Mapping (slam_toolbox)
 - Manual control stack
 - Map saving service for ModeManager
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    blitz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('blitz'),
                'launch',
                'blitz.launch.py'
            )
        )
    )

    odom_node = Node(
        package='rover',
        executable='odom_node',
        name='odom_node',
        output='screen',
        parameters=[
            {'wheel_diameter': 0.11},  # UPDATE: measure your wheel diameter
            {'wheel_base': 0.35},      # UPDATE: measure distance between wheels
            {'ticks_per_rev': 3800},  # Measured: 3800 pulses per wheel rev
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'publish_tf': True}
        ]
    )

    velocity_bridge = Node(
        package='rover',
        executable='velocity_bridge',
        name='velocity_bridge',
        output='screen'
    )


    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    auto_map_saver = Node(
        package='rover',
        executable='auto_map_saver',
        name='auto_map_saver',
        output='screen',
        parameters=[
            {'save_directory': os.path.join(os.path.expanduser('~'), 'maps')}
        ]
    )

    mode_manager = Node(
        package='rover',
        executable='mode_manager',
        name='mode_manager',
        output='screen'
    )

    static_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.18', '0.0', '0.0',
            '0.0', '0.0', '0.0', '1.0',
            'base_link', 'laser'
        ]
    )

    static_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '-0.20', '0.0', '0.0',
            '0.0', '0.0', '0.0', '1.0',
            'base_link', 'imu'
        ]
    )

    return LaunchDescription([
        blitz_launch,
        odom_node,
        static_base_to_laser,
        static_base_to_imu,
        velocity_bridge,
        slam_toolbox_node,
        auto_map_saver,   # <-- service provider
        mode_manager,
    ])
