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
Navigation launch file for my_robot.

This launch file starts the Nav2 navigation stack with AMCL localization
for autonomous navigation using a pre-built map.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for navigation."""
    # Package directories
    nav_pkg = get_package_share_directory('my_robot_navigation')
    sim_pkg = get_package_share_directory('my_robot_simulation')
    
    # Configuration files
    nav2_config_file = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    map_file = os.path.join(nav_pkg, 'maps', 'my_robot_map.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Path to map file'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_config_file,
        description='Path to Nav2 parameters file'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start lifecycle nodes'
    )
    
    # Include robot simulation
    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'combined.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map')
        }]
    )
    
    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    # Nav2 Controller
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    # Nav2 Planner
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    # Nav2 Behaviors
    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    # Nav2 BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    # Nav2 Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    # Nav2 Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    # Lifecycle manager for navigation
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        map_file_arg,
        params_file_arg,
        autostart_arg,
        robot_simulation,
        # Staggered startup for better resource management
        TimerAction(
            period=6.0,  # Reduced delay
            actions=[
                map_server,
                amcl,
            ]
        ),
        TimerAction(
            period=8.0,
            actions=[
                controller,
                planner,
                behaviors,
            ]
        ),
        TimerAction(
            period=10.0,
            actions=[
                bt_navigator,
                waypoint_follower,
                velocity_smoother,
                lifecycle_manager
            ]
        )
    ])