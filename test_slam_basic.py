#!/usr/bin/env python3
"""
简化版SLAM测试脚本
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
import time

class BasicSLAMTest(Node):
    def __init__(self):
        super().__init__('slam_test_node')
        self.get_logger().info("🚀 开始SLAM基础功能测试...")
        
        # 订阅传感器数据
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
            
        # 发布位姿数据
        self.pose_pub = self.create_publisher(
            PoseStamped, '/slam_test/pose', 10)
            
        # 计数器
        self.image_count = 0
        self.depth_count = 0
        self.laser_count = 0
        
        # 测试定时器
        self.test_timer = self.create_timer(5.0, self.report_status)
        
    def image_callback(self, msg):
        self.image_count += 1
        if self.image_count == 1:
            self.get_logger().info(f"✅ RGB图像数据接收正常 - 分辨率: {msg.width}x{msg.height}")
            
    def depth_callback(self, msg):
        self.depth_count += 1 
        if self.depth_count == 1:
            self.get_logger().info(f"✅ 深度图像数据接收正常 - 分辨率: {msg.width}x{msg.height}")
            
    def laser_callback(self, msg):
        self.laser_count += 1
        if self.laser_count == 1:
            self.get_logger().info(f"✅ 激光雷达数据接收正常 - 扫描点数: {len(msg.ranges)}")
            
    def report_status(self):
        self.get_logger().info(f"📊 传感器数据统计:")
        self.get_logger().info(f"   RGB图像: {self.image_count} 帧")
        self.get_logger().info(f"   深度图像: {self.depth_count} 帧") 
        self.get_logger().info(f"   激光数据: {self.laser_count} 帧")
        
        # 发布测试位姿
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

def main():
    rclpy.init()
    node = BasicSLAMTest()
    
    try:
        import threading
        import time
        
        def timeout_shutdown():
            time.sleep(15.0)
            node.get_logger().info("🎉 SLAM基础功能测试完成!")
            rclpy.shutdown()
            
        timer_thread = threading.Thread(target=timeout_shutdown)
        timer_thread.start()
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()