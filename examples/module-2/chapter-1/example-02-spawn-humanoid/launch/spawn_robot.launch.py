#!/usr/bin/env python3
"""Example 02: Spawn Humanoid Robot"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'simple_humanoid.urdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ])
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'simple_humanoid', '-file', urdf_path, '-x', '0', '-y', '0', '-z', '1.0'],
        output='screen'
    )

    return LaunchDescription([gazebo, spawn_entity])
