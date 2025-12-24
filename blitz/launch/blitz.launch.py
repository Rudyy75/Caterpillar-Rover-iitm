from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blitz',
            executable='packer.py',
            name='packer',
            output='screen'
        ),
        Node(
            package='blitz',
            executable='parser.py',
            name='parser',
            output='screen'
        )
    ])
