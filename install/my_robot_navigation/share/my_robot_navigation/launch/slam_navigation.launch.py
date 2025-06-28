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
Complete SLAM and Navigation launch file for my_robot.

This launch file provides a unified system that supports both SLAM mapping 
and autonomous navigation modes, with easy switching between functionalities.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for complete SLAM and navigation system."""
    # Package directories
    nav_pkg = get_package_share_directory('my_robot_navigation')
    sim_pkg = get_package_share_directory('my_robot_simulation')
    desc_pkg = get_package_share_directory('my_robot_description')
    
    # Configuration files
    slam_config_file = os.path.join(nav_pkg, 'config', 'slam_toolbox_config.yaml')
    nav2_config_file = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    map_file = os.path.join(nav_pkg, 'maps', 'my_robot_map.yaml')
    rviz_config_file = os.path.join(desc_pkg, 'rviz', 'navigation_view.rviz')
    
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Operation mode: slam, navigation, or both'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Path to map file for navigation mode'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
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
    
    # SLAM nodes group
    slam_group = GroupAction([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            condition=IfCondition(
                PythonExpression([
                    "('", LaunchConfiguration('mode'), "' == 'slam') or ",
                    "('", LaunchConfiguration('mode'), "' == 'both')"
                ])
            )
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'node_names': ['slam_toolbox']
            }],
            condition=IfCondition(
                PythonExpression([
                    "('", LaunchConfiguration('mode'), "' == 'slam') or ",
                    "('", LaunchConfiguration('mode'), "' == 'both')"
                ])
            )
        )
    ])
    
    # Navigation nodes group
    navigation_group = GroupAction([
        # Map server (only for navigation mode)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': LaunchConfiguration('map')
            }],
            condition=IfCondition(
                PythonExpression([
                    "('", LaunchConfiguration('mode'), "' == 'navigation') or ",
                    "('", LaunchConfiguration('mode'), "' == 'both')"
                ])
            )
        ),
        
        # AMCL (only for navigation mode)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_config_file],
            condition=IfCondition(
                PythonExpression([
                    "('", LaunchConfiguration('mode'), "' == 'navigation') or ",
                    "('", LaunchConfiguration('mode'), "' == 'both')"
                ])
            )
        ),
        
        # Nav2 Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_config_file],
            condition=IfCondition(
                PythonExpression([
                    "('", LaunchConfiguration('mode'), "' == 'navigation') or ",
                    "('", LaunchConfiguration('mode'), "' == 'both')"
                ])
            )
        ),
        
        # Nav2 Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_config_file],
            condition=IfCondition(
                PythonExpression([
                    "('", LaunchConfiguration('mode'), "' == 'navigation') or ",
                    "('", LaunchConfiguration('mode'), "' == 'both')"
                ])
            )
        ),
        
        # Nav2 Behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_config_file],
            condition=IfCondition(
                PythonExpression([
                    "('", LaunchConfiguration('mode'), "' == 'navigation') or ",
                    "('", LaunchConfiguration('mode'), "' == 'both')"
                ])
            )
        ),
        
        # Nav2 BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_config_file],
            condition=IfCondition(
                PythonExpression([
                    "('", LaunchConfiguration('mode'), "' == 'navigation') or ",
                    "('", LaunchConfiguration('mode'), "' == 'both')"
                ])
            )
        ),
        
        # Nav2 Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_config_file],
            condition=IfCondition(
                PythonExpression([
                    "('", LaunchConfiguration('mode'), "' == 'navigation') or ",
                    "('", LaunchConfiguration('mode'), "' == 'both')"
                ])
            )
        ),
        
        # Nav2 Velocity Smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_config_file],
            condition=IfCondition(
                PythonExpression([
                    "('", LaunchConfiguration('mode'), "' == 'navigation') or ",
                    "('", LaunchConfiguration('mode'), "' == 'both')"
                ])
            )
        ),
        
        # Navigation lifecycle manager
        Node(
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
            }],
            condition=IfCondition(
                PythonExpression([
                    "('", LaunchConfiguration('mode'), "' == 'navigation') or ",
                    "('", LaunchConfiguration('mode'), "' == 'both')"
                ])
            )
        )
    ])
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        map_file_arg,
        rviz_arg,
        autostart_arg,
        robot_simulation,
        # Delay SLAM and navigation start to ensure robot is spawned
        TimerAction(
            period=8.0,
            actions=[
                slam_group,
                navigation_group,
                rviz
            ]
        )
    ])