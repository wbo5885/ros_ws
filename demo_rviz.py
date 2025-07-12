#!/usr/bin/env python3
"""
RViz交互式演示脚本
引导用户逐步体验RViz的各种功能
"""

import subprocess
import time
import os
import sys
import signal

class RVizDemo:
    def __init__(self):
        self.workspace_path = "/home/wb/ros_ws"
        self.processes = []
    
    def print_header(self, title):
        """打印标题"""
        print(f"\n{'='*60}")
        print(f"🎯 {title}")
        print(f"{'='*60}")
    
    def print_step(self, step_num, description):
        """打印步骤"""
        print(f"\n📋 步骤 {step_num}: {description}")
        print("-" * 40)
    
    def wait_for_user(self, message="按回车键继续..."):
        """等待用户输入"""
        input(f"\n⏳ {message}")
    
    def run_command_background(self, cmd):
        """后台运行命令"""
        bash_cmd = f"cd {self.workspace_path} && source install/setup.bash && {cmd}"
        process = subprocess.Popen(
            ["bash", "-c", bash_cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        self.processes.append(process)
        return process
    
    def cleanup(self):
        """清理进程"""
        print("\n🧹 清理后台进程...")
        
        # 清理我们的进程
        for process in self.processes:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3)
            except:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except:
                    pass
        
        # 清理可能残留的进程
        commands = ["pkill -f gzserver", "pkill -f gzclient", "pkill -f rviz2", 
                   "pkill -f slam_toolbox", "pkill -f orb_slam3_ai"]
        for cmd in commands:
            subprocess.run(cmd, shell=True, capture_output=True)
        
        self.processes.clear()
        time.sleep(2)
    
    def demo_basic_robot_view(self):
        """演示基本机器人视图"""
        self.print_step(1, "基本机器人视图演示")
        
        print("🤖 这将演示基本的机器人3D模型显示")
        print("📋 功能包括:")
        print("   - 机器人3D模型显示")
        print("   - 激光雷达数据可视化")
        print("   - 相机图像显示")
        print("   - 坐标变换(TF)显示")
        
        self.wait_for_user("准备启动仿真环境...")
        
        # 启动仿真
        print("🚀 启动仿真环境...")
        sim_process = self.run_command_background(
            "ros2 launch my_robot_simulation combined.launch.py gui:=false"
        )
        
        print("⏳ 等待仿真环境启动...")
        time.sleep(8)
        
        self.wait_for_user("现在启动RViz机器人视图...")
        
        # 启动RViz
        print("🎯 启动RViz机器人视图...")
        rviz_cmd = f"cd {self.workspace_path} && source install/setup.bash && rviz2 -d src/my_robot_description/rviz/robot_view.rviz"
        rviz_process = subprocess.Popen(["bash", "-c", rviz_cmd])
        
        print("\n✨ RViz已启动！你应该能看到:")
        print("   🤖 蓝色/白色的机器人3D模型")
        print("   🔴 红色的激光雷达扫描点")
        print("   📱 相机图像窗口")
        print("   🌐 坐标轴显示")
        
        print("\n🎮 你可以:")
        print("   - 鼠标左键: 旋转视角")
        print("   - 鼠标滚轮: 缩放")
        print("   - 鼠标右键: 平移视角")
        
        self.wait_for_user("体验完毕后关闭RViz窗口，然后按回车继续...")
        
        try:
            rviz_process.terminate()
            rviz_process.wait(timeout=5)
        except:
            rviz_process.kill()
    
    def demo_slam_view(self):
        """演示SLAM建图视图"""
        self.print_step(2, "SLAM建图视图演示")
        
        print("🗺️ 这将演示SLAM建图过程的可视化")
        print("📋 功能包括:")
        print("   - 实时地图构建")
        print("   - 机器人轨迹跟踪")
        print("   - 粒子滤波可视化")
        
        self.cleanup()
        self.wait_for_user("准备启动SLAM系统...")
        
        # 启动SLAM
        print("🚀 启动SLAM系统...")
        slam_process = self.run_command_background(
            "ros2 launch my_robot_navigation slam.launch.py"
        )
        
        print("⏳ 等待SLAM系统启动...")
        time.sleep(10)
        
        self.wait_for_user("现在启动RViz SLAM视图...")
        
        # 启动RViz SLAM视图
        print("🎯 启动RViz SLAM视图...")
        rviz_cmd = f"cd {self.workspace_path} && source install/setup.bash && rviz2 -d src/my_robot_description/rviz/slam_view.rviz"
        rviz_process = subprocess.Popen(["bash", "-c", rviz_cmd])
        
        print("\n✨ SLAM视图已启动！你应该能看到:")
        print("   🗺️ 正在构建的地图 (灰色/黑色区域)")
        print("   🤖 机器人当前位置")
        print("   🟢 机器人运动轨迹 (绿色线条)")
        print("   🔴 激光雷达扫描")
        
        print("\n🎮 建议操作:")
        print("   1. 观察地图如何逐渐构建")
        print("   2. 注意机器人轨迹的记录")
        print("   3. 查看激光数据如何形成地图")
        
        print("\n💡 可选: 开启键盘控制来移动机器人:")
        print("   新终端执行: ros2 run turtlebot3_teleop teleop_keyboard")
        
        self.wait_for_user("体验完毕后关闭RViz，然后按回车继续...")
        
        try:
            rviz_process.terminate()
            rviz_process.wait(timeout=5)
        except:
            rviz_process.kill()
    
    def demo_ai_slam_comparison(self):
        """演示AI SLAM对比"""
        self.print_step(3, "AI SLAM对比演示")
        
        print("🤖 这将演示传统SLAM与AI增强SLAM的对比")
        print("📋 功能包括:")
        print("   - 传统SLAM轨迹 (绿色)")
        print("   - AI SLAM轨迹 (红色)")
        print("   - AI优化的地图点")
        print("   - 性能指标监控")
        
        self.wait_for_user("准备启动AI SLAM对比...")
        
        # 启动传统SLAM
        print("🚀 启动传统SLAM...")
        slam_process = self.run_command_background(
            "ros2 launch my_robot_navigation slam.launch.py"
        )
        time.sleep(8)
        
        # 启动AI SLAM
        print("🤖 启动AI SLAM...")
        ai_slam_process = self.run_command_background(
            "ros2 run orb_slam3_ai orb_slam3_ai_node"
        )
        time.sleep(5)
        
        self.wait_for_user("现在启动RViz查看对比效果...")
        
        # 启动RViz
        print("🎯 启动RViz...")
        rviz_cmd = f"cd {self.workspace_path} && source install/setup.bash && rviz2 -d src/my_robot_description/rviz/slam_view.rviz"
        rviz_process = subprocess.Popen(["bash", "-c", rviz_cmd])
        
        print("\n✨ AI SLAM对比视图已启动！")
        print("\n🔧 手动添加AI SLAM显示项:")
        print("   1. 点击 'Add' 按钮")
        print("   2. 添加 'Path' -> Topic: /orb_slam3/path")
        print("   3. 设置颜色为红色以区分传统SLAM")
        print("   4. 添加 'PointCloud2' -> Topic: /orb_slam3/map_points")
        print("   5. 添加 'PoseStamped' -> Topic: /orb_slam3/pose")
        
        print("\n👀 观察要点:")
        print("   🟢 传统SLAM: /map (绿色地图)")
        print("   🔴 AI SLAM轨迹: /orb_slam3/path (红色)")
        print("   ⚫ AI地图点: /orb_slam3/map_points")
        print("   📊 性能监控: /orb_slam3/performance")
        
        print("\n💡 提示:")
        print("   - 可以同时看到两种SLAM的效果")
        print("   - AI SLAM会根据环境自动调整参数")
        print("   - 观察轨迹精度和地图质量的差异")
        
        self.wait_for_user("体验完毕后关闭RViz，然后按回车继续...")
        
        try:
            rviz_process.terminate()
            rviz_process.wait(timeout=5)
        except:
            rviz_process.kill()
    
    def demo_navigation(self):
        """演示导航功能"""
        self.print_step(4, "导航功能演示")
        
        print("🧭 这将演示自主导航功能")
        print("📋 功能包括:")
        print("   - 全局路径规划")
        print("   - 局部路径规划")
        print("   - 代价地图显示")
        print("   - 目标点设置")
        
        print("\n⚠️ 注意: 导航需要预先存在的地图")
        print("   如果没有地图，请先运行SLAM建图并保存")
        
        choice = input("\n❓ 是否继续导航演示? (y/n): ")
        if choice.lower() != 'y':
            print("⏭️ 跳过导航演示")
            return
        
        self.cleanup()
        self.wait_for_user("准备启动导航系统...")
        
        # 启动导航
        print("🚀 启动导航系统...")
        nav_process = self.run_command_background(
            "ros2 launch my_robot_navigation navigation.launch.py"
        )
        
        print("⏳ 等待导航系统启动...")
        time.sleep(10)
        
        self.wait_for_user("现在启动RViz导航视图...")
        
        # 启动RViz导航视图
        print("🎯 启动RViz导航视图...")
        rviz_cmd = f"cd {self.workspace_path} && source install/setup.bash && rviz2 -d src/my_robot_description/rviz/full_navigation.rviz"
        rviz_process = subprocess.Popen(["bash", "-c", rviz_cmd])
        
        print("\n✨ 导航视图已启动！")
        print("\n🎮 交互操作:")
        print("   1. 🎯 '2D Pose Estimate': 设置机器人初始位姿")
        print("      - 点击工具")
        print("      - 在地图上点击机器人实际位置")
        print("      - 拖动设置朝向")
        
        print("\n   2. 🎯 '2D Nav Goal': 设置导航目标")
        print("      - 点击工具")
        print("      - 在地图上点击目标位置")
        print("      - 拖动设置目标朝向")
        print("      - 机器人将自动规划路径并导航")
        
        print("\n👀 观察要点:")
        print("   🗺️ 全局代价地图 (障碍物膨胀)")
        print("   🔵 局部代价地图 (动态更新)")
        print("   🟢 全局规划路径")
        print("   🔴 局部规划路径")
        print("   🤖 机器人实时位姿")
        
        self.wait_for_user("体验完毕后关闭RViz，然后按回车继续...")
        
        try:
            rviz_process.terminate()
            rviz_process.wait(timeout=5)
        except:
            rviz_process.kill()
    
    def run_demo(self):
        """运行完整演示"""
        self.print_header("RViz 交互式功能演示")
        
        print("🎯 本演示将引导您体验RViz的主要功能:")
        print("   1. 基本机器人视图")
        print("   2. SLAM建图视图")
        print("   3. AI SLAM对比")
        print("   4. 导航功能演示")
        
        print("\n💡 提示:")
        print("   - 每个步骤都会有详细说明")
        print("   - 可以随时按Ctrl+C退出")
        print("   - 建议全屏显示以获得最佳体验")
        
        self.wait_for_user("准备开始演示...")
        
        try:
            # 运行各个演示
            self.demo_basic_robot_view()
            self.demo_slam_view()
            self.demo_ai_slam_comparison()
            self.demo_navigation()
            
            # 演示结束
            self.print_header("演示完成")
            print("🎉 恭喜！您已经体验了RViz的主要功能")
            print("\n📚 更多信息:")
            print("   - 查看README.md的RViz使用指南")
            print("   - 运行 python3 test_rviz.py 进行全面测试")
            print("   - 自定义RViz配置: File -> Save Config As...")
            
            print("\n💡 下一步建议:")
            print("   1. 练习使用2D Pose Estimate和2D Nav Goal")
            print("   2. 尝试添加自定义显示项")
            print("   3. 实验不同的RViz配置")
            print("   4. 学习高级功能如Markers和Interactive Markers")
            
        except KeyboardInterrupt:
            print("\n🛑 演示被用户中断")
        except Exception as e:
            print(f"\n❌ 演示过程中发生错误: {e}")
        finally:
            self.cleanup()
            print("\n✅ 清理完成，演示结束")

def main():
    """主函数"""
    if len(sys.argv) > 1 and sys.argv[1] == "--help":
        print("RViz交互式演示脚本")
        print("\n使用方法:")
        print("  python3 demo_rviz.py         # 开始交互式演示")
        print("  python3 demo_rviz.py --help  # 显示帮助")
        print("\n注意事项:")
        print("  - 确保已安装所有必需的ROS包")
        print("  - 建议在有GUI的环境中运行")
        print("  - 演示过程中请保持网络连接")
        return
    
    demo = RVizDemo()
    demo.run_demo()

if __name__ == "__main__":
    main()