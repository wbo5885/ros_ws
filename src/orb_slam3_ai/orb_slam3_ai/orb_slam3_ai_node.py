#!/usr/bin/env python3
"""
ORB-SLAM3 AI优化ROS2节点
集成RGB-D SLAM和AI参数优化
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header, Float32, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import logging
from typing import Dict, Optional
import tf2_ros
from tf2_ros import TransformBroadcaster

# 导入我们的AI优化器
from .slam_ai_optimizer import TrainedSLAMOptimizer, PerformanceMonitor


class ORBSlam3AINode(Node):
    """ORB-SLAM3 AI优化节点"""
    
    def __init__(self):
        super().__init__('orb_slam3_ai_node')
        
        # 设置日志
        logging.basicConfig(level=logging.INFO)
        self.logger = self.get_logger()
        
        # 初始化AI优化器和性能监控器
        self.ai_optimizer = TrainedSLAMOptimizer()
        self.performance_monitor = PerformanceMonitor()
        self.bridge = CvBridge()
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 参数
        self.declare_parameter('optimization_frequency', 0.5)  # Hz
        self.declare_parameter('camera_frame', 'camera_optical_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        
        self.optimization_freq = self.get_parameter('optimization_frequency').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # ROS2订阅者（使用正确的话题名称）
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # ROS2发布者
        self.pose_pub = self.create_publisher(PoseStamped, '/orb_slam3/pose', 10)
        self.path_pub = self.create_publisher(Path, '/orb_slam3/path', 10)
        self.map_points_pub = self.create_publisher(PointCloud2, '/orb_slam3/map_points', 10)
        self.performance_pub = self.create_publisher(Float32, '/orb_slam3/performance', 10)
        self.env_type_pub = self.create_publisher(String, '/orb_slam3/environment_type', 10)
        
        # 数据缓存
        self.current_rgb = None
        self.current_depth = None
        self.camera_info = None
        self.current_motion = {'magnitude': 0.0, 'angular_velocity': 0.0}
        self.last_pose = None
        self.last_optimization_time = time.time()
        
        # SLAM系统状态
        self.slam_initialized = False
        self.current_slam_params = {}
        self.trajectory_path = Path()
        self.trajectory_path.header.frame_id = self.map_frame
        
        # 参数优化定时器
        self.optimization_timer = self.create_timer(
            1.0 / self.optimization_freq, self.optimize_slam_params)
        
        # 性能监控定时器
        self.performance_timer = self.create_timer(1.0, self.publish_performance_metrics)
        
        # 模拟SLAM系统（实际应用中应该替换为真实的ORB-SLAM3接口）
        self.slam_system = self.initialize_mock_slam_system()
        
        self.logger.info("ORB-SLAM3 AI Node initialized successfully")
        self.logger.info(f"Optimization frequency: {self.optimization_freq} Hz")
    
    def initialize_mock_slam_system(self):
        """初始化模拟SLAM系统（实际部署时替换为真实ORB-SLAM3）"""
        class MockSLAMSystem:
            def __init__(self, node_logger):
                self.logger = node_logger
                self.params = {
                    'nFeatures': 1000,
                    'scaleFactor': 1.2,
                    'nLevels': 8,
                    'thRefRatio': 0.75
                }
                self.current_pose = np.eye(4)
                self.map_points = []
                self.is_tracking = True
                
            def update_parameters(self, new_params):
                """更新SLAM参数"""
                self.params.update(new_params)
                self.logger.info(f"SLAM parameters updated: {list(new_params.keys())}")
                
            def process_rgbd_frame(self, rgb_image, depth_image, timestamp):
                """处理RGB-D帧"""
                try:
                    # 模拟SLAM处理
                    # 实际实现中应该调用ORB-SLAM3的TrackRGBD函数
                    
                    # 模拟位姿更新（添加小的随机运动）
                    if self.is_tracking:
                        noise = np.random.normal(0, 0.01, 3)
                        self.current_pose[0:3, 3] += noise
                        
                        # 模拟一些地图点
                        if len(self.map_points) < 1000:
                            new_points = np.random.random((10, 3)) * 5  # 随机点云
                            self.map_points.extend(new_points.tolist())
                    
                    return {
                        'success': self.is_tracking,
                        'pose': self.current_pose.copy(),
                        'num_features': self.params['nFeatures'],
                        'num_map_points': len(self.map_points),
                        'tracking_quality': 0.8 if self.is_tracking else 0.0
                    }
                    
                except Exception as e:
                    self.logger.error(f"SLAM processing error: {e}")
                    return {'success': False, 'pose': np.eye(4), 'num_features': 0}
                    
            def get_map_points(self):
                """获取地图点"""
                return np.array(self.map_points) if self.map_points else np.empty((0, 3))
                
            def reset(self):
                """重置SLAM系统"""
                self.current_pose = np.eye(4)
                self.map_points = []
                self.is_tracking = True
                self.logger.info("SLAM system reset")
        
        return MockSLAMSystem(self.logger)
    
    def rgb_callback(self, msg):
        """RGB图像回调"""
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.logger.error(f"RGB image conversion error: {e}")
    
    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.logger.error(f"Depth image conversion error: {e}")
    
    def camera_info_callback(self, msg):
        """相机信息回调"""
        self.camera_info = msg
    
    def odom_callback(self, msg):
        """里程计回调"""
        try:
            # 计算运动幅度
            if self.last_pose is not None:
                dx = msg.pose.pose.position.x - self.last_pose.pose.pose.position.x
                dy = msg.pose.pose.position.y - self.last_pose.pose.pose.position.y
                dz = msg.pose.pose.position.z - self.last_pose.pose.pose.position.z
                self.current_motion['magnitude'] = np.sqrt(dx**2 + dy**2 + dz**2)
            
            self.current_motion['angular_velocity'] = abs(msg.twist.twist.angular.z)
            self.last_pose = msg
            
        except Exception as e:
            self.logger.error(f"Odometry processing error: {e}")
    
    def optimize_slam_params(self):
        """AI参数优化主函数"""
        if self.current_rgb is None:
            self.logger.debug("等待RGB图像数据...")
            return
        
        try:
            current_time = time.time()
            
            # AI参数优化
            optimized_params, env_type, env_features = self.ai_optimizer.optimize_parameters(
                self.current_rgb, self.current_motion
            )
            
            # 更新SLAM系统参数
            self.slam_system.update_parameters(optimized_params)
            self.current_slam_params = optimized_params
            
            # 处理RGB-D帧
            if self.current_depth is not None:
                slam_result = self.slam_system.process_rgbd_frame(
                    self.current_rgb, self.current_depth, current_time
                )
                
                if slam_result['success']:
                    # 发布位姿
                    self.publish_pose(slam_result['pose'], current_time)
                    
                    # 更新轨迹
                    self.update_trajectory(slam_result['pose'], current_time)
                    
                    # 发布地图点
                    self.publish_map_points()
                    
                    # 评估性能
                    performance_score = self.evaluate_slam_performance(slam_result)
                    self.ai_optimizer.update_performance_feedback(performance_score)
            
            # 发布环境类型
            env_msg = String()
            env_msg.data = env_type
            self.env_type_pub.publish(env_msg)
            
            # 记录优化信息
            self.logger.info(
                f"Env: {env_type}, "
                f"Brightness: {env_features.get('brightness', 0):.2f}, "
                f"Texture: {env_features.get('texture_density', 0):.2f}, "
                f"Motion: {env_features.get('motion_magnitude', 0):.2f}"
            )
            
        except Exception as e:
            self.logger.error(f"SLAM optimization error: {e}")
    
    def publish_pose(self, pose_matrix: np.ndarray, timestamp: float):
        """发布SLAM位姿"""
        try:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.map_frame
            
            # 从4x4矩阵提取位置和旋转
            pose_msg.pose.position.x = float(pose_matrix[0, 3])
            pose_msg.pose.position.y = float(pose_matrix[1, 3])
            pose_msg.pose.position.z = float(pose_matrix[2, 3])
            
            # 从旋转矩阵计算四元数
            from scipy.spatial.transform import Rotation
            rotation_matrix = pose_matrix[0:3, 0:3]
            r = Rotation.from_matrix(rotation_matrix)
            quat = r.as_quat()  # [x, y, z, w]
            
            pose_msg.pose.orientation.x = float(quat[0])
            pose_msg.pose.orientation.y = float(quat[1])
            pose_msg.pose.orientation.z = float(quat[2])
            pose_msg.pose.orientation.w = float(quat[3])
            
            self.pose_pub.publish(pose_msg)
            
            # 发布TF变换
            self.publish_tf_transform(pose_matrix, timestamp)
            
        except Exception as e:
            self.logger.error(f"Pose publishing error: {e}")
    
    def publish_tf_transform(self, pose_matrix: np.ndarray, timestamp: float):
        """发布TF变换"""
        try:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.map_frame
            transform.child_frame_id = self.camera_frame
            
            # 设置平移
            transform.transform.translation.x = float(pose_matrix[0, 3])
            transform.transform.translation.y = float(pose_matrix[1, 3])
            transform.transform.translation.z = float(pose_matrix[2, 3])
            
            # 设置旋转
            from scipy.spatial.transform import Rotation
            rotation_matrix = pose_matrix[0:3, 0:3]
            r = Rotation.from_matrix(rotation_matrix)
            quat = r.as_quat()
            
            transform.transform.rotation.x = float(quat[0])
            transform.transform.rotation.y = float(quat[1])
            transform.transform.rotation.z = float(quat[2])
            transform.transform.rotation.w = float(quat[3])
            
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            self.logger.error(f"TF transform error: {e}")
    
    def update_trajectory(self, pose_matrix: np.ndarray, timestamp: float):
        """更新轨迹路径"""
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = self.map_frame
            
            pose_stamped.pose.position.x = float(pose_matrix[0, 3])
            pose_stamped.pose.position.y = float(pose_matrix[1, 3])
            pose_stamped.pose.position.z = float(pose_matrix[2, 3])
            
            from scipy.spatial.transform import Rotation
            rotation_matrix = pose_matrix[0:3, 0:3]
            r = Rotation.from_matrix(rotation_matrix)
            quat = r.as_quat()
            
            pose_stamped.pose.orientation.x = float(quat[0])
            pose_stamped.pose.orientation.y = float(quat[1])
            pose_stamped.pose.orientation.z = float(quat[2])
            pose_stamped.pose.orientation.w = float(quat[3])
            
            self.trajectory_path.poses.append(pose_stamped)
            
            # 限制轨迹长度
            if len(self.trajectory_path.poses) > 1000:
                self.trajectory_path.poses.pop(0)
            
            self.trajectory_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.trajectory_path)
            
        except Exception as e:
            self.logger.error(f"Trajectory update error: {e}")
    
    def publish_map_points(self):
        """发布地图点云"""
        try:
            map_points = self.slam_system.get_map_points()
            
            if len(map_points) > 0:
                # 创建点云消息（简化版本）
                # 实际实现中应该使用sensor_msgs/PointCloud2
                self.logger.debug(f"Map contains {len(map_points)} points")
                
        except Exception as e:
            self.logger.error(f"Map points publishing error: {e}")
    
    def evaluate_slam_performance(self, slam_result: Dict) -> float:
        """评估SLAM性能"""
        try:
            tracking_quality = slam_result.get('tracking_quality', 0.5)
            num_features = slam_result.get('num_features', 0)
            num_map_points = slam_result.get('num_map_points', 0)
            
            # 综合性能评分
            feature_score = min(1.0, num_features / 1000.0)
            mapping_score = min(1.0, num_map_points / 5000.0)
            
            performance = 0.5 * tracking_quality + 0.3 * feature_score + 0.2 * mapping_score
            return performance
            
        except Exception as e:
            self.logger.error(f"Performance evaluation error: {e}")
            return 0.5
    
    def publish_performance_metrics(self):
        """发布性能指标"""
        try:
            performance_stats = self.ai_optimizer.get_performance_stats()
            
            performance_msg = Float32()
            performance_msg.data = performance_stats.get('avg_performance', 0.0)
            self.performance_pub.publish(performance_msg)
            
            # 日志性能信息
            sample_count = performance_stats.get('sample_count', 0)
            if sample_count > 0:
                self.logger.info(
                    f"Performance - Avg: {performance_stats.get('avg_performance', 0.0):.3f}, "
                    f"Trend: {performance_stats.get('recent_trend', 0.0):.3f}, "
                    f"Samples: {sample_count}"
                )
                
        except Exception as e:
            self.logger.error(f"Performance metrics error: {e}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = ORBSlam3AINode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()