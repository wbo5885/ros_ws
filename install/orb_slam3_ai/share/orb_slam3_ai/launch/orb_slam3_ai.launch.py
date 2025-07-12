#!/usr/bin/env python3
"""
ORB-SLAM3 AI优化系统启动文件
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成ORB-SLAM3 AI系统的launch描述"""
    
    # 包目录
    orb_slam3_ai_dir = get_package_share_directory('orb_slam3_ai')
    robot_simulation_dir = get_package_share_directory('my_robot_simulation')
    robot_description_dir = get_package_share_directory('my_robot_description')
    
    # 配置文件路径
    orb_slam3_config = os.path.join(orb_slam3_ai_dir, 'config', 'orb_slam3_params.yaml')
    rviz_config = os.path.join(orb_slam3_ai_dir, 'rviz', 'orb_slam3_ai.rviz')
    
    # Launch参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    optimization_freq_arg = DeclareLaunchArgument(
        'optimization_frequency',
        default_value='0.5',
        description='AI optimization frequency in Hz'
    )
    
    # 包含机器人仿真（带RGB-D相机）
    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_simulation'),
                'launch',
                'combined.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui')
        }.items()
    )
    
    # ORB-SLAM3 AI节点
    orb_slam3_ai_node = Node(
        package='orb_slam3_ai',
        executable='orb_slam3_ai_node',
        name='orb_slam3_ai',
        output='screen',
        parameters=[
            orb_slam3_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'optimization_frequency': LaunchConfiguration('optimization_frequency')
            }
        ],
        remappings=[
            # 话题重映射（如需要）
        ]
    )
    
    # RViz可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # 性能监控节点（可选）
    performance_monitor = Node(
        package='orb_slam3_ai',
        executable='performance_monitor',
        name='performance_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # 静态TF发布器 - 相机到base_link的变换
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            '0.2', '0', '0.15',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'base_link', 'camera_link'
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        optimization_freq_arg,
        
        # 机器人仿真
        robot_simulation,
        
        # 静态变换
        static_tf_camera,
        
        # 延迟启动SLAM系统，确保仿真环境已就绪
        TimerAction(
            period=5.0,
            actions=[
                GroupAction([
                    orb_slam3_ai_node,
                    # performance_monitor,  # 可选启用
                ])
            ]
        ),
        
        # 延迟启动RViz
        TimerAction(
            period=8.0,
            actions=[rviz_node]
        )
    ])