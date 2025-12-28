#!/usr/bin/env python3
"""
Autonomous Mode Launch File (Map-Based Nav2)

Behavior:
 - DOES NOT launch slam_toolbox
 - DOES NOT publish any SLAM-related TFs
 - Prompts user to enter a map number at launch time
 - Loads <number>.yaml / <number>.pgm saved earlier by auto_map_saver
 - Launches Nav2 + RViz using that map
"""

import os

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


MAPS_DIR = os.path.join(os.path.expanduser('~'), 'maps')


def launch_nav2_with_selected_map(context, *args, **kwargs):
    # Ask user for map index
    map_index = input(
        '\nEnter map number to load for Nav2 (e.g. 1, 2, 3): '
    ).strip()

    if not map_index.isdigit():
        raise RuntimeError(
            f'Invalid input "{map_index}". Expected a numeric map index.'
        )

    yaml_path = os.path.join(MAPS_DIR, f'{map_index}.yaml')
    pgm_path = os.path.join(MAPS_DIR, f'{map_index}.pgm')

    if not os.path.exists(yaml_path) or not os.path.exists(pgm_path):
        raise RuntimeError(
            f'Map files not found:\n  {yaml_path}\n  {pgm_path}'
        )

    print(f'\nUsing map: {yaml_path}\n')

    # Nav2 bringup launch
    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'map': yaml_path,
            'use_sim_time': 'False',
            'autostart': 'True'
        }.items()
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return [
        LogInfo(msg=f'Launching Nav2 using map {map_index}.yaml'),
        nav2,
        rviz
    ]


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        LogInfo(
            msg='Autonomous mode: SLAM disabled. Waiting for user to select saved map.'
        )
    )

    # Interactive map selection + Nav2 launch
    ld.add_action(
        OpaqueFunction(function=launch_nav2_with_selected_map)
    )

    return ld
