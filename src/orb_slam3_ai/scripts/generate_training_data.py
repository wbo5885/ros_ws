#!/usr/bin/env python3
"""
从仿真环境生成ORB-SLAM3 AI训练数据
使用当前的ROS2机器人系统收集训练样本
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import cv2
import numpy as np
import json
import os
import time
from datetime import datetime
from pathlib import Path
from cv_bridge import CvBridge
import threading
import random

class TrainingDataCollector(Node):
    """训练数据收集器"""
    
    def __init__(self):
        super().__init__('training_data_collector')
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 数据保存目录
        self.data_dir = Path('/home/wb/ros_ws/slam_training_data/simulation_data')
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
        (self.data_dir / 'images').mkdir(exist_ok=True)
        (self.data_dir / 'depth').mkdir(exist_ok=True)
        
        # 订阅传感器话题
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # 发布运动控制
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 数据存储
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_laser = None
        self.latest_odom = None
        
        # 训练样本
        self.training_samples = []
        self.sample_count = 0
        
        # 收集参数
        self.collection_interval = 2.0  # 每2秒收集一个样本
        self.max_samples = 500  # 最大样本数
        
        # 运动模式
        self.motion_patterns = [
            {'linear': 0.2, 'angular': 0.0, 'duration': 3.0, 'env_type': 'normal'},
            {'linear': 0.0, 'angular': 0.3, 'duration': 2.0, 'env_type': 'fast_motion'},
            {'linear': 0.1, 'angular': 0.1, 'duration': 4.0, 'env_type': 'normal'},
            {'linear': 0.0, 'angular': 0.0, 'duration': 2.0, 'env_type': 'static'},
            {'linear': -0.1, 'angular': 0.0, 'duration': 2.0, 'env_type': 'normal'},
        ]
        
        self.current_motion_idx = 0
        self.motion_start_time = time.time()
        
        # 创建定时器
        self.collection_timer = self.create_timer(self.collection_interval, self.collect_sample)
        self.motion_timer = self.create_timer(0.1, self.update_motion)
        
        self.get_logger().info("🚀 训练数据收集器启动")
        self.get_logger().info(f"📁 数据保存到: {self.data_dir}")
    
    def rgb_callback(self, msg):
        """RGB图像回调"""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"RGB转换失败: {e}")
    
    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().warn(f"深度图转换失败: {e}")
    
    def laser_callback(self, msg):
        """激光雷达回调"""
        self.latest_laser = msg
    
    def odom_callback(self, msg):
        """里程计回调"""
        self.latest_odom = msg
    
    def update_motion(self):
        """更新机器人运动"""
        if self.sample_count >= self.max_samples:
            return
        
        current_time = time.time()
        motion = self.motion_patterns[self.current_motion_idx]
        
        # 检查是否需要切换运动模式
        if current_time - self.motion_start_time > motion['duration']:
            self.current_motion_idx = (self.current_motion_idx + 1) % len(self.motion_patterns)
            self.motion_start_time = current_time
            motion = self.motion_patterns[self.current_motion_idx]
            
            self.get_logger().info(f"🔄 切换运动模式: {motion}")
        
        # 发布运动命令
        cmd = Twist()
        cmd.linear.x = motion['linear']
        cmd.angular.z = motion['angular']
        self.cmd_pub.publish(cmd)
    
    def collect_sample(self):
        """收集一个训练样本"""
        if self.sample_count >= self.max_samples:
            self.get_logger().info(f"✅ 达到最大样本数: {self.max_samples}")
            self.save_training_data()
            rclpy.shutdown()
            return
        
        # 检查数据完整性
        if not all([self.latest_rgb is not None, 
                   self.latest_depth is not None,
                   self.latest_laser is not None,
                   self.latest_odom is not None]):
            self.get_logger().warn("⚠️ 传感器数据不完整，跳过此样本")
            return
        
        try:
            # 生成样本ID
            sample_id = f"sim_{self.sample_count:04d}_{int(time.time())}"
            
            # 保存图像
            rgb_filename = f"{sample_id}_rgb.png"
            depth_filename = f"{sample_id}_depth.png"
            
            cv2.imwrite(str(self.data_dir / 'images' / rgb_filename), self.latest_rgb)
            cv2.imwrite(str(self.data_dir / 'depth' / depth_filename), self.latest_depth)
            
            # 分析环境特征
            env_features = self.analyze_environment_features()
            
            # 获取当前运动状态
            motion = self.motion_patterns[self.current_motion_idx]
            env_type = motion['env_type']
            
            # 生成SLAM参数 (基于环境类型)
            optimal_params = self.generate_optimal_params(env_type, env_features)
            
            # 计算性能评分 (基于特征质量)
            performance_score = self.calculate_performance_score(env_features)
            
            # 创建训练样本
            sample = {
                "sample_id": sample_id,
                "image_path": f"images/{rgb_filename}",
                "depth_path": f"depth/{depth_filename}",
                "timestamp": time.time(),
                "env_features": env_features,
                "optimal_params": optimal_params,
                "performance_score": performance_score,
                "scene_type": env_type,
                "motion_type": self.get_motion_type(motion),
                "robot_pose": self.get_robot_pose(),
                "laser_stats": self.get_laser_stats()
            }
            
            self.training_samples.append(sample)
            self.sample_count += 1
            
            self.get_logger().info(f"📊 收集样本 {self.sample_count}/{self.max_samples}: {env_type}, 性能={performance_score:.3f}")
            
            # 定期保存数据
            if self.sample_count % 50 == 0:
                self.save_training_data()
                
        except Exception as e:
            self.get_logger().error(f"❌ 收集样本失败: {e}")
    
    def analyze_environment_features(self) -> list:
        """分析环境特征"""
        gray = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2GRAY)
        
        # 亮度
        brightness = np.mean(gray) / 255.0
        
        # 对比度
        contrast = np.std(gray) / 255.0
        
        # 纹理密度
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        edge_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        texture_density = np.mean(edge_magnitude) / 255.0
        
        # 运动幅度 (从里程计)
        if self.latest_odom:
            motion_magnitude = abs(self.latest_odom.twist.twist.linear.x)
            angular_velocity = abs(self.latest_odom.twist.twist.angular.z)
        else:
            motion_magnitude = 0.0
            angular_velocity = 0.0
        
        # 边缘密度
        edges = cv2.Canny(gray, 50, 150)
        edge_density = np.sum(edges > 0) / (edges.shape[0] * edges.shape[1])
        
        # 特征点密度
        orb = cv2.ORB_create()
        keypoints = orb.detect(gray, None)
        feature_density = len(keypoints) / 1000.0
        
        # 稳定性 (基于图像质量和运动)
        stability = min(1.0, contrast * 2.0) * (1.0 - motion_magnitude)
        
        return [
            brightness,
            contrast,
            texture_density,
            motion_magnitude,
            angular_velocity,
            edge_density,
            feature_density,
            stability
        ]
    
    def generate_optimal_params(self, env_type: str, env_features: list) -> list:
        """根据环境类型和特征生成最优SLAM参数"""
        base_configs = {
            'normal': [1000, 1.2, 8, 0.75, 0.9, 0.25, 0.05, 3],
            'high_texture': [1200, 1.25, 8, 0.7, 0.9, 0.3, 0.05, 4],
            'low_texture': [600, 1.1, 6, 0.85, 0.8, 0.2, 0.1, 2],
            'low_light': [500, 1.15, 6, 0.8, 0.75, 0.25, 0.08, 3],
            'fast_motion': [1000, 1.2, 7, 0.75, 0.85, 0.35, 0.06, 3],
            'static': [800, 1.15, 7, 0.8, 0.95, 0.2, 0.04, 4]
        }
        
        base_params = base_configs.get(env_type, base_configs['normal'])
        
        # 根据环境特征进行微调
        brightness, contrast, texture_density = env_features[:3]
        
        adjusted_params = base_params.copy()
        
        # 根据亮度调整
        if brightness < 0.3:  # 低光照
            adjusted_params[0] = max(400, adjusted_params[0] * 0.8)  # 减少特征点
            adjusted_params[3] = min(0.9, adjusted_params[3] + 0.1)  # 提高阈值
        
        # 根据纹理密度调整
        if texture_density > 0.5:  # 高纹理
            adjusted_params[0] = min(1500, adjusted_params[0] * 1.2)  # 增加特征点
            adjusted_params[1] = min(1.3, adjusted_params[1] + 0.05)  # 增加尺度因子
        
        # 添加小幅随机扰动增加数据多样性
        for i in range(len(adjusted_params)):
            noise = np.random.normal(0, 0.03)  # 3%噪声
            adjusted_params[i] = max(0.1, adjusted_params[i] * (1 + noise))
        
        return adjusted_params
    
    def calculate_performance_score(self, env_features: list) -> float:
        """计算性能评分"""
        brightness, contrast, texture_density, motion, angular_vel, edge_density, feature_density, stability = env_features
        
        # 基础评分
        base_score = 0.7
        
        # 图像质量评分
        image_quality = (contrast * 0.4 + texture_density * 0.3 + edge_density * 0.3)
        
        # 运动稳定性评分
        motion_stability = max(0, 1.0 - motion * 2.0 - angular_vel * 1.5)
        
        # 特征丰富度评分
        feature_richness = min(1.0, feature_density)
        
        # 综合评分
        total_score = base_score + image_quality * 0.2 + motion_stability * 0.1 + feature_richness * 0.1
        
        return min(1.0, max(0.0, total_score))
    
    def get_motion_type(self, motion: dict) -> str:
        """获取运动类型"""
        if motion['linear'] == 0 and motion['angular'] == 0:
            return 'static'
        elif abs(motion['linear']) > 0.15:
            return 'fast_translation'
        elif abs(motion['angular']) > 0.25:
            return 'fast_rotation'
        else:
            return 'slow_motion'
    
    def get_robot_pose(self) -> dict:
        """获取机器人位姿"""
        if self.latest_odom:
            pose = self.latest_odom.pose.pose
            return {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
                'qw': pose.orientation.w
            }
        return {}
    
    def get_laser_stats(self) -> dict:
        """获取激光雷达统计"""
        if self.latest_laser:
            ranges = np.array(self.latest_laser.ranges)
            valid_ranges = ranges[np.isfinite(ranges) & (ranges > 0)]
            
            return {
                'min_range': float(np.min(valid_ranges)) if len(valid_ranges) > 0 else 0.0,
                'max_range': float(np.max(valid_ranges)) if len(valid_ranges) > 0 else 0.0,
                'mean_range': float(np.mean(valid_ranges)) if len(valid_ranges) > 0 else 0.0,
                'std_range': float(np.std(valid_ranges)) if len(valid_ranges) > 0 else 0.0,
                'valid_points': len(valid_ranges),
                'total_points': len(ranges)
            }
        return {}
    
    def save_training_data(self):
        """保存训练数据"""
        if not self.training_samples:
            return
        
        # 保存JSON文件
        output_file = self.data_dir / 'training_data.json'
        with open(output_file, 'w') as f:
            json.dump(self.training_samples, f, indent=2)
        
        # 创建数据集信息
        dataset_info = {
            'created_at': datetime.now().isoformat(),
            'total_samples': len(self.training_samples),
            'data_directory': str(self.data_dir),
            'collection_parameters': {
                'collection_interval': self.collection_interval,
                'max_samples': self.max_samples,
                'motion_patterns': self.motion_patterns
            },
            'statistics': self.calculate_dataset_statistics()
        }
        
        info_file = self.data_dir / 'dataset_info.json'
        with open(info_file, 'w') as f:
            json.dump(dataset_info, f, indent=2)
        
        self.get_logger().info(f"💾 训练数据已保存:")
        self.get_logger().info(f"   样本数: {len(self.training_samples)}")
        self.get_logger().info(f"   数据文件: {output_file}")
        self.get_logger().info(f"   信息文件: {info_file}")
    
    def calculate_dataset_statistics(self) -> dict:
        """计算数据集统计信息"""
        if not self.training_samples:
            return {}
        
        scene_types = {}
        motion_types = {}
        total_score = 0
        
        for sample in self.training_samples:
            scene_type = sample['scene_type']
            motion_type = sample['motion_type']
            
            scene_types[scene_type] = scene_types.get(scene_type, 0) + 1
            motion_types[motion_type] = motion_types.get(motion_type, 0) + 1
            total_score += sample['performance_score']
        
        return {
            'scene_type_distribution': scene_types,
            'motion_type_distribution': motion_types,
            'average_performance_score': total_score / len(self.training_samples),
            'total_samples': len(self.training_samples)
        }

def main():
    rclpy.init()
    
    try:
        collector = TrainingDataCollector()
        
        print("🚀 开始从仿真环境收集训练数据...")
        print("   - 机器人将自动执行各种运动模式")
        print("   - 每2秒收集一个训练样本")
        print("   - 预计收集500个样本")
        print("   - 按Ctrl+C提前停止")
        
        rclpy.spin(collector)
        
    except KeyboardInterrupt:
        print("\\n⚠️ 用户中断收集")
    except Exception as e:
        print(f"\\n❌ 收集过程出错: {e}")
    finally:
        if 'collector' in locals():
            collector.save_training_data()
        rclpy.shutdown()

if __name__ == '__main__':
    main()