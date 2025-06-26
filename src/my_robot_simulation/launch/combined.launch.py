import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sim = 'my_robot_simulation'
    pkg_robot = 'my_robot_description'

    sim_share_dir = get_package_share_directory(pkg_sim)
    robot_share_dir = get_package_share_directory(pkg_robot)

    # 路径：竞技场launch文件 & 机器人URDF文件
    arena_launch_file = os.path.join(sim_share_dir, 'launch', 'my_hex_arena.launch.py')
    robot_urdf_file = os.path.join(robot_share_dir, 'urdf', 'model.urdf')

    # 启动竞技场
    arena_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(arena_launch_file)
    )

    # 5秒后动态加载机器人
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'my_robot',
                    '-file', robot_urdf_file,
                    '-x', '0', '-y', '0', '-z', '0.3'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        arena_launch,
        spawn_robot
    ])

