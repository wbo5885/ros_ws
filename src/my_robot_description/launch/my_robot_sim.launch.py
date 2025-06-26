from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    world_path = os.path.join(pkg_share, 'worlds', 'my_robot_world.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'model.urdf')

    gazebo_plugin_path = '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                world_path,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
            ],
            output='screen',
            additional_env={'GAZEBO_PLUGIN_PATH': gazebo_plugin_path}
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=['-entity', 'my_robot', '-file', urdf_path],
                    output='screen'
                )
            ]
        )
    ])
