#!/usr/bin/env python3
"""
RViz快速测试脚本 - 简化版本
快速验证RViz基本功能和配置文件
"""

import subprocess
import time
import os
import sys

def run_command(cmd, timeout=10):
    """执行命令"""
    try:
        bash_cmd = f"cd /home/wb/ros_ws && source install/setup.bash && {cmd}"
        result = subprocess.run(
            ["bash", "-c", bash_cmd],
            timeout=timeout,
            capture_output=True,
            text=True
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "超时"
    except Exception as e:
        return False, "", str(e)

def check_file(file_path):
    """检查文件是否存在"""
    full_path = f"/home/wb/ros_ws/{file_path}"
    return os.path.exists(full_path)

def main():
    print("🧪 RViz 快速测试")
    print("=" * 40)
    
    # 1. 检查RViz配置文件
    print("\n📁 检查RViz配置文件...")
    configs = [
        "src/my_robot_description/rviz/robot_view.rviz",
        "src/my_robot_description/rviz/slam_view.rviz", 
        "src/my_robot_description/rviz/navigation_view.rviz",
        "src/my_robot_description/rviz/full_navigation.rviz"
    ]
    
    for config in configs:
        if check_file(config):
            print(f"✅ {os.path.basename(config)}")
        else:
            print(f"❌ {os.path.basename(config)} - 文件不存在")
    
    # 2. 检查ROS包
    print("\n📦 检查ROS包...")
    packages = ["my_robot_description", "my_robot_simulation", "my_robot_navigation", "orb_slam3_ai"]
    
    for package in packages:
        success, stdout, stderr = run_command(f"ros2 pkg list | grep {package}")
        if success and package in stdout:
            print(f"✅ {package}")
        else:
            print(f"❌ {package} - 包未找到")
    
    # 3. 快速启动测试
    print("\n🚀 快速启动测试...")
    
    # 启动仿真环境
    print("启动仿真环境...")
    sim_process = subprocess.Popen(
        ["bash", "-c", "cd /home/wb/ros_ws && source install/setup.bash && ros2 launch my_robot_simulation combined.launch.py gui:=false"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    
    # 等待启动
    time.sleep(8)
    
    # 检查话题
    success, stdout, stderr = run_command("ros2 topic list")
    if success:
        topics = stdout.split('\n')
        required_topics = ['/scan', '/odom', '/camera/image_raw']
        found_topics = [topic for topic in required_topics if topic in topics]
        
        print(f"找到话题: {len(found_topics)}/{len(required_topics)}")
        for topic in found_topics:
            print(f"✅ {topic}")
        
        missing = [topic for topic in required_topics if topic not in found_topics]
        for topic in missing:
            print(f"❌ {topic} - 缺失")
    
    # 测试RViz启动
    print("\n🎯 测试RViz启动...")
    rviz_process = subprocess.Popen(
        ["bash", "-c", "cd /home/wb/ros_ws && source install/setup.bash && timeout 10 rviz2 -d src/my_robot_description/rviz/robot_view.rviz"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    
    # 等待RViz启动
    time.sleep(5)
    
    # 检查RViz是否运行
    check_success, check_stdout, check_stderr = run_command("pgrep rviz2")
    if check_success and check_stdout:
        print("✅ RViz启动成功")
    else:
        print("❌ RViz启动失败")
    
    # 清理进程
    print("\n🧹 清理进程...")
    subprocess.run("pkill -f gzserver", shell=True, capture_output=True)
    subprocess.run("pkill -f gzclient", shell=True, capture_output=True)
    subprocess.run("pkill -f rviz2", shell=True, capture_output=True)
    
    try:
        sim_process.terminate()
        sim_process.wait(timeout=5)
    except:
        sim_process.kill()
    
    try:
        rviz_process.terminate()
        rviz_process.wait(timeout=5)
    except:
        rviz_process.kill()
    
    print("\n✅ 快速测试完成！")
    print("\n💡 如需详细测试，请运行: python3 test_rviz.py")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 测试被中断")
        subprocess.run("pkill -f gzserver", shell=True, capture_output=True)
        subprocess.run("pkill -f gzclient", shell=True, capture_output=True)
        subprocess.run("pkill -f rviz2", shell=True, capture_output=True)