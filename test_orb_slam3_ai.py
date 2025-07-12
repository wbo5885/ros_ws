#!/usr/bin/env python3
"""
ORB-SLAM3 AI系统测试脚本
"""

import rclpy
from rclpy.node import Node
import time
import subprocess
import sys

class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')
        self.get_logger().info("开始测试ORB-SLAM3 AI系统...")
        
    def test_basic_functionality(self):
        """测试基本功能"""
        try:
            # 测试包导入
            from orb_slam3_ai.slam_ai_optimizer import SimpleSLAMOptimizer
            self.get_logger().info("✅ AI优化器导入成功")
            
            # 测试AI优化器初始化
            optimizer = SimpleSLAMOptimizer()
            self.get_logger().info("✅ AI优化器初始化成功")
            
            # 测试环境分析
            import numpy as np
            test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            motion_data = {'magnitude': 0.1, 'angular_velocity': 0.2}
            
            params, env_type, features = optimizer.optimize_parameters(test_image, motion_data)
            self.get_logger().info(f"✅ 参数优化测试成功 - 环境类型: {env_type}")
            self.get_logger().info(f"   优化参数: {list(params.keys())}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ 测试失败: {e}")
            return False

def main():
    """主测试函数"""
    rclpy.init()
    
    try:
        tester = SystemTester()
        
        # 测试基本功能
        if tester.test_basic_functionality():
            tester.get_logger().info("🎉 所有测试通过！系统就绪。")
            
            # 提供启动指令
            tester.get_logger().info("\n" + "="*50)
            tester.get_logger().info("启动完整系统的命令:")
            tester.get_logger().info("source install/setup.bash")
            tester.get_logger().info("ros2 launch orb_slam3_ai orb_slam3_ai.launch.py")
            tester.get_logger().info("="*50)
        else:
            tester.get_logger().error("❌ 测试失败，请检查配置")
            
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()