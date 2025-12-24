"""
Autonomous Mode Launch File
Launched dynamically by mode_manager when autonomous mode is activated.

Nodes (placeholder - add your actual autonomous nodes):
    - navigation (TODO)
    - ml_pipeline (TODO)
    - slam (TODO)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    # ============ PLACEHOLDER NODES ============
    # Replace these with your actual autonomous stack nodes
    
    # Example: Navigation node
    # nav_node = Node(
    #     package='nav2_bringup',
    #     executable='navigation_launch',
    #     name='navigation',
    #     output='screen'
    # )
    
    # Example: ML pipeline node
    # ml_node = Node(
    #     package='rover',
    #     executable='ml_pipeline',
    #     name='ml_pipeline',
    #     output='screen'
    # )
    
    # Example: SLAM node
    # slam_node = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam',
    #     output='screen'
    # )
    
    return LaunchDescription([
        LogInfo(msg='Autonomous stack launched!'),
        LogInfo(msg='TODO: Add ML, SLAM, and Navigation nodes here'),
        
        # Uncomment and add your actual nodes:
        # nav_node,
        # ml_node,
        # slam_node,
    ])
