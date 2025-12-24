"""
Manual Mode Launch File
Launches the base stack needed for both manual and autonomous operation.

Nodes:
    - blitz (packer + parser) - Serial communication with ESP32
    - odom_node - Odometry from encoders + BNO
    - mode_manager - Listens for mode switch, launches autonomous stack
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
            {'wheel_diameter': 0.1},
            {'wheel_base': 0.3},
            {'ticks_per_rev': 360}
        ]
    )
    
    mode_manager = Node(
        package='rover',
        executable='mode_manager',
        name='mode_manager',
        output='screen'
    )
    
    return LaunchDescription([
        blitz_launch,
        odom_node,
        mode_manager,
    ])
