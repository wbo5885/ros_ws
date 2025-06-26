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
Launch file for my_robot basic simulation.

This launch file starts Gazebo with the default world and spawns the robot.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for robot simulation."""
    # Package directories
    pkg_share = get_package_share_directory('my_robot_description')
    
    # File paths
    world_file = os.path.join(pkg_share, 'worlds', 'my_robot_world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'model.urdf')
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to world file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Launch Gazebo
    gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            LaunchConfiguration('world'),
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': open(urdf_file).read()},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Spawn robot entity
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                arguments=[
                    '-entity', 'my_robot',
                    '-file', urdf_file,
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gazebo_cmd,
        robot_state_publisher,
        spawn_robot
    ])