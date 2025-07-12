#!/usr/bin/env python3
"""
RViz显示问题修复脚本
解决"只看到三个坐标轴"的问题
"""

import subprocess
import time
import os

def run_command(cmd):
    """执行命令"""
    try:
        bash_cmd = f"cd /home/wb/ros_ws && source install/setup.bash && {cmd}"
        result = subprocess.run(["bash", "-c", bash_cmd], capture_output=True, text=True, timeout=10)
        return result.returncode == 0, result.stdout, result.stderr
    except:
        return False, "", "超时"

def check_system_status():
    """检查系统状态"""
    print("🔍 检查系统状态...")
    
    # 检查话题
    success, stdout, stderr = run_command("ros2 topic list")
    if not success:
        print("❌ ROS2系统未运行！请先启动仿真环境")
        return False
    
    topics = stdout.split('\n')
    
    # 检查关键话题
    required_topics = {
        '/robot_description': '机器人模型',
        '/tf': 'TF变换',
        '/scan': '激光雷达',
        '/odom': '里程计'
    }
    
    print("\n📡 话题检查:")
    all_good = True
    for topic, desc in required_topics.items():
        if topic in topics:
            print(f"✅ {topic} - {desc}")
        else:
            print(f"❌ {topic} - {desc} (缺失)")
            all_good = False
    
    return all_good

def get_available_frames():
    """获取可用的坐标系"""
    print("\n🔍 检查可用的坐标系...")
    
    success, stdout, stderr = run_command("timeout 5 ros2 topic echo /tf_static --once")
    if success and 'frame_id' in stdout:
        print("✅ TF系统正常")
    else:
        print("⚠️ TF系统可能有问题")
    
    # 检查tf树
    success, stdout, stderr = run_command("timeout 5 ros2 run tf2_ros tf2_echo odom base_link")
    if success:
        print("✅ odom -> base_link 变换正常")
        return ['odom', 'base_link']
    else:
        print("⚠️ 基础TF变换有问题")
        return ['odom']

def create_fixed_rviz_config():
    """创建修复的RViz配置"""
    print("\n🔧 创建修复的RViz配置...")
    
    # 获取可用坐标系
    frames = get_available_frames()
    fixed_frame = frames[0] if frames else 'odom'
    
    config_content = f"""Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /RobotModel1
        - /LaserScan1
      Splitter Ratio: 0.5
    Tree Height: 557
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Mass Properties:
        Inertia: false
        Mass: false
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 25; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Filter (blacklist): ""
      Filter (whitelist): ""
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Alpha: 1
      Marker Scale: 0.3
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        {{}}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {fixed_frame}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
Views:
  Current:
    Class: rviz_default_plugins/Orbit
    Distance: 5.855692386627197
    Enable Stereo Rendering:
      Stereo Eye Separation: 0.05999999865889549
      Stereo Focal Distance: 1
      Swap Stereo Eyes: false
      Value: false
    Focal Point:
      X: 0
      Y: 0
      Z: 0
    Focal Shape Fixed Size: true
    Focal Shape Size: 0.05000000074505806
    Invert Z Axis: false
    Name: Current View
    Near Clip Distance: 0.009999999776482582
    Pitch: 0.4603982269763947
    Target Frame: <Fixed Frame>
    Value: Orbit (rviz_default_plugins)
    Yaw: 0.785398006439209
  Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002b0fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002b0000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500670065007400730200000153000000a5000001ad000000d4fb0000000c004b006900740063006800650063006f000001150000014c0000000000000000000000010000010f000002b0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002b0000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004b00000003efc0100000002fb0000000800540069006d00650100000000000004b0000002fb00fffffffb0000000800540069006d00650100000000000004500000000000000000000002590000002b00000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 60
  Y: 60"""
    
    # 保存配置
    config_path = "/home/wb/ros_ws/fixed_robot_view.rviz"
    with open(config_path, 'w') as f:
        f.write(config_content)
    
    print(f"✅ 创建修复配置: {config_path}")
    print(f"🎯 Fixed Frame 设置为: {fixed_frame}")
    
    return config_path

def main():
    """主函数"""
    print("🔧 RViz显示问题修复工具")
    print("=" * 40)
    
    # 检查系统状态
    if not check_system_status():
        print("\n❌ 系统状态异常，请先解决基础问题")
        print("\n💡 建议操作:")
        print("1. 确保仿真环境正在运行:")
        print("   ros2 launch my_robot_simulation combined.launch.py")
        print("2. 检查话题是否正常:")
        print("   ros2 topic list")
        return
    
    # 创建修复配置
    config_path = create_fixed_rviz_config()
    
    print("\n🎯 解决方案:")
    print("1. 关闭当前的RViz窗口")
    print(f"2. 使用修复的配置启动RViz:")
    print(f"   rviz2 -d {config_path}")
    
    print("\n🔧 如果仍有问题，手动检查以下设置:")
    print("- Global Options -> Fixed Frame 设为 'odom' 或 'base_link'")
    print("- 确保 RobotModel 的 Description Topic 设为 '/robot_description'")
    print("- 确保 LaserScan 的 Topic 设为 '/scan'")
    print("- 检查各显示项的 Enabled 选项是否勾选")
    
    print("\n📋 常见Fixed Frame选项:")
    print("- odom: 里程计坐标系 (推荐)")
    print("- base_link: 机器人本体坐标系")
    print("- map: 地图坐标系 (SLAM时使用)")
    
    # 询问是否直接启动
    choice = input("\n❓ 是否立即使用修复配置启动RViz? (y/n): ")
    if choice.lower() == 'y':
        print("🚀 启动修复的RViz配置...")
        os.system(f"cd /home/wb/ros_ws && source install/setup.bash && rviz2 -d {config_path}")

if __name__ == "__main__":
    main()