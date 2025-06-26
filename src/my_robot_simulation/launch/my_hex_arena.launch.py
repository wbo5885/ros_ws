import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess # 用于直接执行外部命令 (gzserver, gzclient)

def generate_launch_description():
    pkg_my_robot_simulation = get_package_share_directory('my_robot_simulation')

    # 设置你的自定义世界文件路径
    world_file_name = 'my_hex_arena.world'
    world_path = os.path.join(pkg_my_robot_simulation, 'worlds', world_file_name)

    # 1. 启动 Gazebo 服务器 (gzserver)，并明确指定加载你的世界文件
    # '-s' 参数用于加载 Gazebo 插件，这些是ROS-Gazebo集成所必需的
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_force_system.so'],
        output='screen', # 将输出显示在屏幕上
        emulate_tty=True # 模拟TTY，以更好地显示颜色和输出
    )

    # 2. 启动 Gazebo 客户端 (gzclient)，即 Gazebo 的图形界面
    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        emulate_tty=True
    )

    # 返回 LaunchDescription，现在只包含直接启动 gzserver 和 gzclient 的命令
    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
    ])

