#!/usr/bin/env python3
"""
RViz测试脚本 - 验证RViz配置和功能
自动测试所有预配置的RViz视图和相关功能
"""

import subprocess
import time
import os
import sys
import signal
import psutil
from datetime import datetime

class Colors:
    """终端颜色定义"""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    PURPLE = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    BOLD = '\033[1m'
    END = '\033[0m'

class RVizTester:
    def __init__(self):
        self.workspace_path = "/home/wb/ros_ws"
        self.processes = []
        self.test_results = {}
        
    def log(self, message, level="INFO"):
        """彩色日志输出"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        colors = {
            "INFO": Colors.WHITE,
            "SUCCESS": Colors.GREEN,
            "ERROR": Colors.RED,
            "WARNING": Colors.YELLOW,
            "TEST": Colors.CYAN
        }
        color = colors.get(level, Colors.WHITE)
        print(f"{color}[{timestamp}] {level}: {message}{Colors.END}")
    
    def run_command(self, cmd, timeout=15, background=False):
        """执行命令"""
        try:
            bash_cmd = f"cd {self.workspace_path} && source install/setup.bash && {cmd}"
            
            if background:
                process = subprocess.Popen(
                    ["bash", "-c", bash_cmd],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                self.processes.append(process)
                return process, None, None
            else:
                result = subprocess.run(
                    ["bash", "-c", bash_cmd],
                    timeout=timeout,
                    capture_output=True,
                    text=True
                )
                return None, result.stdout, result.stderr
                
        except subprocess.TimeoutExpired:
            return None, None, "Command timeout"
        except Exception as e:
            return None, None, str(e)
    
    def cleanup_processes(self):
        """清理所有后台进程"""
        self.log("清理后台进程...", "INFO")
        
        # 清理我们启动的进程
        for process in self.processes:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
            except:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except:
                    pass
        
        # 清理可能残留的进程
        commands = [
            "pkill -f gzserver",
            "pkill -f gzclient", 
            "pkill -f rviz2",
            "pkill -f robot_state_publisher",
            "pkill -f slam_toolbox",
            "pkill -f orb_slam3_ai"
        ]
        
        for cmd in commands:
            subprocess.run(cmd, shell=True, capture_output=True)
        
        self.processes.clear()
        time.sleep(2)
    
    def check_rviz_configs(self):
        """检查RViz配置文件"""
        self.log("检查RViz配置文件...", "TEST")
        
        config_files = [
            "src/my_robot_description/rviz/robot_view.rviz",
            "src/my_robot_description/rviz/slam_view.rviz", 
            "src/my_robot_description/rviz/navigation_view.rviz",
            "src/my_robot_description/rviz/full_navigation.rviz"
        ]
        
        for config_file in config_files:
            full_path = os.path.join(self.workspace_path, config_file)
            if os.path.exists(full_path):
                self.log(f"✅ {config_file}", "SUCCESS")
                self.test_results[f"config_{os.path.basename(config_file)}"] = True
            else:
                self.log(f"❌ {config_file} - 文件不存在", "ERROR")
                self.test_results[f"config_{os.path.basename(config_file)}"] = False
    
    def check_ros_packages(self):
        """检查ROS包是否正确安装"""
        self.log("检查ROS包安装...", "TEST")
        
        packages = [
            "my_robot_description",
            "my_robot_simulation", 
            "my_robot_navigation",
            "my_robot_vision",
            "orb_slam3_ai"
        ]
        
        for package in packages:
            _, stdout, stderr = self.run_command(f"ros2 pkg list | grep {package}")
            if stdout and package in stdout:
                self.log(f"✅ {package} 包已安装", "SUCCESS")
                self.test_results[f"package_{package}"] = True
            else:
                self.log(f"❌ {package} 包未找到", "ERROR")
                self.test_results[f"package_{package}"] = False
    
    def test_basic_simulation(self):
        """测试基础仿真环境"""
        self.log("测试基础仿真环境...", "TEST")
        
        try:
            # 启动仿真
            process, _, _ = self.run_command(
                "ros2 launch my_robot_simulation combined.launch.py gui:=false", 
                background=True
            )
            
            # 等待启动
            time.sleep(8)
            
            # 检查话题
            _, stdout, stderr = self.run_command("ros2 topic list")
            
            required_topics = ["/scan", "/odom", "/camera/image_raw", "/camera/depth/image_raw"]
            topics_found = []
            
            if stdout:
                for topic in required_topics:
                    if topic in stdout:
                        topics_found.append(topic)
            
            if len(topics_found) == len(required_topics):
                self.log(f"✅ 仿真环境正常，找到 {len(topics_found)} 个必需话题", "SUCCESS")
                self.test_results["simulation"] = True
            else:
                self.log(f"❌ 仿真环境异常，只找到 {len(topics_found)}/{len(required_topics)} 个话题", "ERROR")
                self.test_results["simulation"] = False
            
            return process
            
        except Exception as e:
            self.log(f"❌ 仿真环境启动失败: {e}", "ERROR")
            self.test_results["simulation"] = False
            return None
    
    def test_rviz_startup(self, config_name, config_path, simulation_process=None):
        """测试RViz启动"""
        self.log(f"测试RViz配置: {config_name}", "TEST")
        
        try:
            # 启动RViz
            rviz_cmd = f"timeout 15 rviz2 -d {config_path}"
            process, _, _ = self.run_command(rviz_cmd, background=True)
            
            # 等待RViz启动
            time.sleep(5)
            
            # 检查RViz进程是否在运行
            rviz_running = False
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if 'rviz2' in proc.info['name'] or any('rviz2' in cmd for cmd in proc.info['cmdline'] or []):
                        rviz_running = True
                        break
                except:
                    continue
            
            if rviz_running:
                self.log(f"✅ RViz {config_name} 启动成功", "SUCCESS")
                self.test_results[f"rviz_{config_name}"] = True
            else:
                self.log(f"❌ RViz {config_name} 启动失败", "ERROR")
                self.test_results[f"rviz_{config_name}"] = False
            
            # 停止RViz
            if process:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.wait(timeout=5)
                except:
                    pass
            
            # 确保清理RViz进程
            subprocess.run("pkill -f rviz2", shell=True, capture_output=True)
            time.sleep(2)
            
        except Exception as e:
            self.log(f"❌ RViz {config_name} 测试失败: {e}", "ERROR")
            self.test_results[f"rviz_{config_name}"] = False
    
    def test_slam_integration(self):
        """测试SLAM与RViz集成"""
        self.log("测试SLAM与RViz集成...", "TEST")
        
        try:
            # 启动SLAM
            slam_process, _, _ = self.run_command(
                "ros2 launch my_robot_navigation slam.launch.py",
                background=True
            )
            
            # 等待SLAM启动
            time.sleep(10)
            
            # 检查SLAM话题
            _, stdout, stderr = self.run_command("ros2 topic list")
            
            slam_topics = ["/map", "/tf", "/tf_static"]
            topics_found = 0
            
            if stdout:
                for topic in slam_topics:
                    if topic in stdout:
                        topics_found += 1
            
            if topics_found >= 2:  # 至少要有map和tf
                self.log("✅ SLAM系统正常运行", "SUCCESS")
                
                # 测试SLAM视图
                self.test_rviz_startup(
                    "slam_view", 
                    "src/my_robot_description/rviz/slam_view.rviz",
                    slam_process
                )
                self.test_results["slam_integration"] = True
            else:
                self.log("❌ SLAM系统启动异常", "ERROR")
                self.test_results["slam_integration"] = False
            
        except Exception as e:
            self.log(f"❌ SLAM集成测试失败: {e}", "ERROR")
            self.test_results["slam_integration"] = False
    
    def test_ai_slam_integration(self):
        """测试AI SLAM与RViz集成"""
        self.log("测试AI SLAM与RViz集成...", "TEST")
        
        try:
            # 启动基础仿真
            sim_process, _, _ = self.run_command(
                "ros2 launch my_robot_simulation combined.launch.py gui:=false",
                background=True
            )
            time.sleep(8)
            
            # 启动AI SLAM
            ai_slam_process, _, _ = self.run_command(
                "timeout 20 ros2 run orb_slam3_ai orb_slam3_ai_node",
                background=True
            )
            
            # 等待AI SLAM启动
            time.sleep(10)
            
            # 检查AI SLAM话题
            _, stdout, stderr = self.run_command("ros2 topic list")
            
            ai_slam_topics = ["/orb_slam3/pose", "/orb_slam3/path", "/orb_slam3/performance"]
            topics_found = 0
            
            if stdout:
                for topic in ai_slam_topics:
                    if topic in stdout:
                        topics_found += 1
            
            if topics_found >= 1:  # 至少要有一个AI SLAM话题
                self.log("✅ AI SLAM系统正常运行", "SUCCESS")
                self.test_results["ai_slam_integration"] = True
            else:
                self.log("❌ AI SLAM系统启动异常", "ERROR")
                self.test_results["ai_slam_integration"] = False
            
        except Exception as e:
            self.log(f"❌ AI SLAM集成测试失败: {e}", "ERROR")
            self.test_results["ai_slam_integration"] = False
    
    def test_performance(self):
        """测试系统性能"""
        self.log("测试系统性能...", "TEST")
        
        try:
            # 启动完整系统
            process, _, _ = self.run_command(
                "ros2 launch my_robot_simulation combined.launch.py gui:=false",
                background=True
            )
            time.sleep(10)
            
            # 检查话题频率
            topics_to_check = ["/scan", "/odom", "/camera/image_raw"]
            performance_ok = True
            
            for topic in topics_to_check:
                _, stdout, stderr = self.run_command(f"timeout 5 ros2 topic hz {topic}")
                if stderr and "timeout" in stderr.lower():
                    self.log(f"⚠️  {topic} 话题频率检测超时", "WARNING")
                    performance_ok = False
                elif stdout and any(char.isdigit() for char in stdout):
                    self.log(f"✅ {topic} 话题正常发布", "SUCCESS")
                else:
                    self.log(f"❌ {topic} 话题异常", "ERROR")
                    performance_ok = False
            
            self.test_results["performance"] = performance_ok
            
        except Exception as e:
            self.log(f"❌ 性能测试失败: {e}", "ERROR")
            self.test_results["performance"] = False
    
    def generate_report(self):
        """生成测试报告"""
        self.log("生成测试报告...", "INFO")
        
        total_tests = len(self.test_results)
        passed_tests = sum(self.test_results.values())
        
        print(f"\n{Colors.BOLD}{'='*60}")
        print(f"🧪 RViz 测试报告")
        print(f"{'='*60}{Colors.END}")
        
        print(f"\n📊 总体结果:")
        print(f"   通过: {Colors.GREEN}{passed_tests}{Colors.END}")
        print(f"   失败: {Colors.RED}{total_tests - passed_tests}{Colors.END}")
        print(f"   总计: {total_tests}")
        print(f"   成功率: {Colors.GREEN if passed_tests/total_tests > 0.8 else Colors.YELLOW}{(passed_tests/total_tests)*100:.1f}%{Colors.END}")
        
        print(f"\n📋 详细结果:")
        for test_name, result in self.test_results.items():
            status = f"{Colors.GREEN}✅ PASS{Colors.END}" if result else f"{Colors.RED}❌ FAIL{Colors.END}"
            print(f"   {test_name:<25} {status}")
        
        # 保存报告到文件
        report_file = "rviz_test_report.txt"
        with open(report_file, 'w') as f:
            f.write(f"RViz测试报告 - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("="*60 + "\n\n")
            f.write(f"总体结果: {passed_tests}/{total_tests} 通过 ({(passed_tests/total_tests)*100:.1f}%)\n\n")
            f.write("详细结果:\n")
            for test_name, result in self.test_results.items():
                f.write(f"  {test_name}: {'PASS' if result else 'FAIL'}\n")
        
        print(f"\n📄 详细报告已保存到: {Colors.CYAN}{report_file}{Colors.END}")
        
        if passed_tests == total_tests:
            print(f"\n🎉 {Colors.GREEN}所有测试通过！RViz系统完全正常！{Colors.END}")
        elif passed_tests / total_tests > 0.8:
            print(f"\n⚠️  {Colors.YELLOW}大部分测试通过，系统基本正常{Colors.END}")
        else:
            print(f"\n❌ {Colors.RED}多项测试失败，需要检查系统配置{Colors.END}")
    
    def run_all_tests(self):
        """运行所有测试"""
        print(f"{Colors.BOLD}{Colors.CYAN}")
        print("🧪 RViz 全面测试开始")
        print("=" * 50)
        print(f"{Colors.END}")
        
        try:
            # 1. 基础检查
            self.check_rviz_configs()
            self.check_ros_packages()
            
            # 2. 仿真测试
            simulation_process = self.test_basic_simulation()
            
            # 3. RViz配置测试
            if simulation_process:
                configs = [
                    ("robot_view", "src/my_robot_description/rviz/robot_view.rviz"),
                    ("navigation_view", "src/my_robot_description/rviz/navigation_view.rviz")
                ]
                
                for config_name, config_path in configs:
                    self.test_rviz_startup(config_name, config_path, simulation_process)
                
                self.cleanup_processes()
            
            # 4. SLAM集成测试
            self.test_slam_integration()
            self.cleanup_processes()
            
            # 5. AI SLAM集成测试
            self.test_ai_slam_integration()
            self.cleanup_processes()
            
            # 6. 性能测试
            self.test_performance()
            self.cleanup_processes()
            
            # 7. 生成报告
            self.generate_report()
            
        except KeyboardInterrupt:
            self.log("\n🛑 测试被用户中断", "WARNING")
        except Exception as e:
            self.log(f"❌ 测试过程中发生错误: {e}", "ERROR")
        finally:
            self.cleanup_processes()

def main():
    """主函数"""
    print(f"{Colors.BOLD}🎯 RViz 自动化测试工具{Colors.END}")
    print(f"📅 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"📁 工作目录: /home/wb/ros_ws\n")
    
    if len(sys.argv) > 1 and sys.argv[1] == "--help":
        print("使用方法:")
        print("  python3 test_rviz.py         # 运行完整测试")
        print("  python3 test_rviz.py --help  # 显示帮助")
        return
    
    tester = RVizTester()
    
    try:
        tester.run_all_tests()
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}测试被中断{Colors.END}")
        tester.cleanup_processes()

if __name__ == "__main__":
    main()