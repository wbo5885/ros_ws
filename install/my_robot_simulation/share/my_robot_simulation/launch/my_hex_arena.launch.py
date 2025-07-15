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
Launch file for hexagonal arena simulation environment.

This launch file starts Gazebo with the hexagonal arena world.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description for hexagonal arena."""
    # Get package share directory
    pkg_simulation = get_package_share_directory('my_robot_simulation')
    
    # Set the path to your custom world file
    world_file = os.path.join(pkg_simulation, 'worlds', 'my_hex_arena.world')
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to world file'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run headless'
    )
    
    # 1. Launch Gazebo server (gzserver) with specified world file
    # '-s' parameter loads Gazebo plugins required for ROS-Gazebo integration
    gzserver_cmd = ExecuteProcess(
        cmd=[
            'gzserver', 
            LaunchConfiguration('world'),
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_force_system.so'
        ],
        output='screen',  # Display output on screen
        emulate_tty=True  # Emulate TTY for better color and output display
    )
    
    # 2. Launch Gazebo client (gzclient) - the graphical interface
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('gui'))  # 只有 gui:=true 时才启动
    )
    
    # Return LaunchDescription with commands to launch gzserver and gzclient
    return LaunchDescription([
        world_arg,
        gui_arg,
        gzserver_cmd,
        gzclient_cmd,
    ])