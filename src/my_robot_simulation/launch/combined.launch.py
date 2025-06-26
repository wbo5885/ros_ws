#!/usr/bin/env python3
# Copyright 2024 wb
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Combined launch file for robot simulation in hexagonal arena.

This launch file starts the hexagonal arena and spawns the robot with
robot state publisher for complete simulation.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    TimerAction, 
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for combined simulation."""
    # Package names
    pkg_sim = 'my_robot_simulation'
    pkg_robot = 'my_robot_description'
    
    # Package directories
    sim_share_dir = get_package_share_directory(pkg_sim)
    robot_share_dir = get_package_share_directory(pkg_robot)
    
    # Paths: Arena launch file & Robot URDF file
    arena_launch_file = os.path.join(
        sim_share_dir, 'launch', 'my_hex_arena.launch.py'
    )
    robot_urdf_file = os.path.join(robot_share_dir, 'urdf', 'model.urdf')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='X coordinate for robot spawn'
    )
    
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', 
        default_value='0.0',
        description='Y coordinate for robot spawn'
    )
    
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.3',
        description='Z coordinate for robot spawn'
    )
    
    # Launch the arena
    arena_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(arena_launch_file)
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': open(robot_urdf_file).read()},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Dynamically spawn robot after 5 seconds
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                arguments=[
                    '-entity', 'my_robot',
                    '-file', robot_urdf_file,
                    '-x', LaunchConfiguration('spawn_x'),
                    '-y', LaunchConfiguration('spawn_y'),
                    '-z', LaunchConfiguration('spawn_z')
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        arena_launch,
        robot_state_publisher,
        spawn_robot
    ])