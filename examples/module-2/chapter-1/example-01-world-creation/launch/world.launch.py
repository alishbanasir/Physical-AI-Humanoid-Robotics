#!/usr/bin/env python3
"""
Launch file for Example 01: Basic World Creation
Demonstrates Gazebo physics configuration and empty world setup
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to world file
    world_file = os.path.join(
        os.path.dirname(__file__),
        '..',
        'worlds',
        'empty_world.world'
    )

    # Launch Gazebo with empty_world.world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'world': world_file}.items()
    )

    return LaunchDescription([
        gazebo
    ])
