#!/usr/bin/env python3
"""
ORB-SLAM3 AI参数优化器
基于训练好的深度学习模型动态调整SLAM参数
"""

import torch
import torch.nn as nn
import numpy as np
import cv2
import os
from typing import Dict, Tuple, List
import logging


class ImprovedSLAMNetwork(nn.Module):
    """改进的SLAM参数预测网络"""
    
    def __init__(self, env_feature_dim=8, output_dim=8):
        super().__init__()
        
        # CNN特征提取器
        self.feature_extractor = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.MaxPool2d(2, 2),
            
            nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(2, 2),
            
            nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((4, 4)),
            
            nn.Flatten(),
            nn.Linear(128 * 4 * 4, 256),
            nn.ReLU(),
            nn.Dropout(0.3)
        )
        
        # 参数预测网络
        self.param_predictor = nn.Sequential(
            nn.Linear(256 + env_feature_dim, 128),
            nn.ReLU(),
            nn.BatchNorm1d(128),
            nn.Dropout(0.2),
            
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.BatchNorm1d(64),
            nn.Dropout(0.1),
            
            nn.Linear(64, output_dim)
        )
    
    def forward(self, image, env_features):
        img_features = self.feature_extractor(image)
        combined_features = torch.cat([img_features, env_features], dim=1)
        param_adjustments = self.param_predictor(combined_features)
        return param_adjustments


class TrainedSLAMOptimizer:
    """基于训练好的深度学习模型的SLAM参数优化器"""
    
    def __init__(self, device='cpu'):
        self.device = device
        self.logger = logging.getLogger(__name__)
        
        # 模型和统计信息
        self.model = None
        self.param_stats = None
        
        # 尝试加载训练好的模型
        self._load_trained_model()
        
        # 环境状态缓存
        self.motion_history = []
        self.performance_history = []
        self.max_history_length = 50
        
        self.logger.info("Trained SLAM AI Optimizer initialized")
    
    def _load_trained_model(self):
        """加载训练好的模型"""
        model_path = '/home/wb/ros_ws/src/orb_slam3_ai/models/trained/best_slam_model.pth'
        
        if os.path.exists(model_path):
            try:
                checkpoint = torch.load(model_path, map_location=self.device, weights_only=False)
                
                # 创建模型
                self.model = ImprovedSLAMNetwork()
                self.model.load_state_dict(checkpoint['model_state_dict'])
                self.model.eval()
                self.model.to(self.device)
                
                # 加载参数归一化统计信息
                self.param_stats = checkpoint.get('param_stats', None)
                
                self.logger.info(f"✅ Loaded trained model from {model_path}")
                return True
                
            except Exception as e:
                self.logger.error(f"❌ Failed to load trained model: {e}")
                
        # 如果没有训练好的模型，创建简单的备用模型
        self._create_fallback_model()
        return False
    
    def _create_fallback_model(self):
        """创建备用的简单模型"""
        self.logger.info("🔄 Creating fallback heuristic model")
        
        # 简单的启发式参数映射
        self.heuristic_params = {
            'low_texture': [600, 1.1, 6, 0.85, 0.8, 0.2, 0.1, 2],
            'high_texture': [1200, 1.25, 8, 0.7, 0.9, 0.3, 0.05, 4],
            'low_light': [500, 1.15, 6, 0.8, 0.75, 0.25, 0.08, 3],
            'fast_motion': [1000, 1.2, 7, 0.75, 0.85, 0.35, 0.06, 3],
            'normal': [1000, 1.2, 8, 0.75, 0.9, 0.25, 0.05, 3]
        }
        
        # 默认参数统计（用于归一化）
        self.param_stats = {
            'mean': np.array([9.71359495e+02, 1.19849759e+00, 7.69821142e+00, 7.56592183e-01,
                             9.10700207e-01, 2.57012206e-01, 1.00000000e-01, 3.13179926e+00]),
            'std': np.array([7.53698896e+01, 3.34664960e-02, 5.87870023e-01, 2.66347496e-02,
                            4.09307485e-02, 4.18624427e-02, 1.00000000e-08, 3.09718186e-01])
        }
    
    def normalize_params(self, params):
        """归一化参数"""
        if self.param_stats is None:
            return params
        return (params - self.param_stats['mean']) / self.param_stats['std']
    
    def denormalize_params(self, normalized_params):
        """反归一化参数"""
        if self.param_stats is None:
            return normalized_params
        return normalized_params * self.param_stats['std'] + self.param_stats['mean']
        # 这些权重基于ORB-SLAM3的最佳实践经验
        self.heuristic_params = {
            'low_texture': {
                'nFeatures': 600,
                'scaleFactor': 1.1,
                'nLevels': 6,
                'thRefRatio': 0.85,
                'KeyFrameLinearity': 0.8,
                'KeyFrameAngularity': 0.2,
                'minScore': 0.1,
                'nSimilarityGroups': 2
            },
            'high_texture': {
                'nFeatures': 1200,
                'scaleFactor': 1.25,
                'nLevels': 8,
                'thRefRatio': 0.7,
                'KeyFrameLinearity': 0.9,
                'KeyFrameAngularity': 0.3,
                'minScore': 0.05,
                'nSimilarityGroups': 4
            },
            'low_light': {
                'nFeatures': 500,
                'scaleFactor': 1.15,
                'nLevels': 6,
                'thRefRatio': 0.8,
                'KeyFrameLinearity': 0.75,
                'KeyFrameAngularity': 0.25,
                'minScore': 0.08,
                'nSimilarityGroups': 3
            },
            'fast_motion': {
                'nFeatures': 1000,
                'scaleFactor': 1.2,
                'nLevels': 7,
                'thRefRatio': 0.75,
                'KeyFrameLinearity': 0.85,
                'KeyFrameAngularity': 0.35,
                'minScore': 0.06,
                'nSimilarityGroups': 3
            },
            'normal': {
                'nFeatures': 1000,
                'scaleFactor': 1.2,
                'nLevels': 8,
                'thRefRatio': 0.75,
                'KeyFrameLinearity': 0.9,
                'KeyFrameAngularity': 0.3,
                'minScore': 0.05,
                'nSimilarityGroups': 3
            }
        }
    
    def preprocess_image(self, image: np.ndarray) -> torch.Tensor:
        """预处理图像用于模型输入"""
        try:
            # 确保图像是RGB格式
            if len(image.shape) == 3:
                if image.shape[2] == 3:
                    # BGR to RGB
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                # 灰度图转RGB
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
            
            # 调整尺寸到64x64 (训练时使用的尺寸)
            image = cv2.resize(image, (64, 64))
            
            # 归一化到[0,1]
            image = image.astype(np.float32) / 255.0
            
            # 转换为PyTorch张量格式 (CHW)
            image_tensor = torch.from_numpy(np.transpose(image, (2, 0, 1)))
            image_tensor = image_tensor.unsqueeze(0).to(self.device)  # 添加batch维度
            
            return image_tensor
            
        except Exception as e:
            self.logger.error(f"Image preprocessing error: {e}")
            # 返回默认的64x64 RGB图像
            default_image = torch.zeros(1, 3, 64, 64, device=self.device)
            return default_image
    
    def analyze_environment(self, image: np.ndarray, motion_data: Dict) -> Dict[str, float]:
        """分析环境特征"""
        try:
            # 转换为灰度图进行分析
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image
            
            # 基础图像特征
            brightness = np.mean(gray) / 255.0
            contrast = np.std(gray) / 255.0
            
            # 纹理特征 - ORB角点检测
            orb = cv2.ORB_create(nfeatures=1000)
            keypoints = orb.detect(gray, None)
            texture_density = min(len(keypoints) / 1000.0, 1.0)  # 归一化
            
            # 运动特征
            motion_magnitude = motion_data.get('magnitude', 0.0)
            angular_velocity = abs(motion_data.get('angular_velocity', 0.0))
            
            # 梯度特征（边缘密度）
            grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
            grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
            gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
            edge_density = np.mean(gradient_magnitude > 50)
            
            # 光流密度（运动纹理）
            motion_texture = min(motion_magnitude * texture_density, 1.0)
            
            return {
                'brightness': brightness,
                'contrast': contrast,
                'texture_density': texture_density,
                'motion_magnitude': motion_magnitude,
                'angular_velocity': angular_velocity,
                'edge_density': edge_density,
                'motion_texture': motion_texture,
                'stability': 1.0 - min(motion_magnitude + angular_velocity, 1.0)
            }
            
        except Exception as e:
            self.logger.error(f"Environment analysis error: {e}")
            return {
                'brightness': 0.5, 'contrast': 0.5, 'texture_density': 0.5,
                'motion_magnitude': 0.0, 'angular_velocity': 0.0,
                'edge_density': 0.5, 'motion_texture': 0.5, 'stability': 1.0
            }
    
    def classify_environment_type(self, env_features: Dict[str, float]) -> str:
        """基于特征分类环境类型"""
        
        # 多条件环境分类
        if env_features['brightness'] < 0.3:
            return 'low_light'
        elif env_features['texture_density'] < 0.2:
            return 'low_texture'
        elif env_features['texture_density'] > 0.8 and env_features['edge_density'] > 0.7:
            return 'high_texture'
        elif env_features['motion_magnitude'] > 0.3 or env_features['angular_velocity'] > 0.5:
            return 'fast_motion'
        else:
            return 'normal'
    
    def optimize_parameters(self, image: np.ndarray, motion_data: Dict) -> Tuple[Dict[str, float], str, Dict[str, float]]:
        """优化SLAM参数"""
        
        try:
            # 1. 分析环境特征
            env_features = self.analyze_environment(image, motion_data)
            
            # 2. 分类环境类型
            env_type = self.classify_environment_type(env_features)
            
            # 3. 使用训练好的模型或启发式方法
            if self.model is not None:
                # 使用训练好的深度学习模型
                optimized_params = self._optimize_with_trained_model(image, env_features)
            else:
                # 使用启发式方法
                optimized_params = self._optimize_with_heuristics(env_features)
            
            return optimized_params, env_type, env_features
            
        except Exception as e:
            self.logger.error(f"Parameter optimization error: {e}")
            default_env_features = {
                'brightness': 0.5, 'contrast': 0.5, 'texture_density': 0.5,
                'motion_magnitude': 0.0, 'angular_velocity': 0.0,
                'edge_density': 0.5, 'motion_texture': 0.5, 'stability': 1.0
            }
            return self._get_default_params(), 'normal', default_env_features
    
    def _optimize_with_trained_model(self, image: np.ndarray, env_features: Dict) -> Dict:
        """使用训练好的模型进行参数优化"""
        try:
            # 预处理图像
            image_tensor = self.preprocess_image(image)
            
            # 准备环境特征向量
            env_feature_vector = torch.tensor([
                env_features['brightness'],
                env_features['contrast'],
                env_features['texture_density'],
                env_features['motion_magnitude'],
                env_features['angular_velocity'],
                env_features['edge_density'],
                env_features['motion_texture'],
                env_features['stability']
            ], dtype=torch.float32, device=self.device).unsqueeze(0)
            
            # 模型预测
            with torch.no_grad():
                normalized_params = self.model(image_tensor, env_feature_vector)
                # 反归一化得到实际参数
                predicted_params = self.denormalize_params(normalized_params.squeeze().cpu().numpy())
            
            # 转换为参数字典
            param_names = ['nFeatures', 'scaleFactor', 'nLevels', 'thRefRatio', 
                          'KeyFrameLinearity', 'KeyFrameAngularity', 'minScore', 'nSimilarityGroups']
            
            optimized_params = {}
            for i, param_name in enumerate(param_names):
                val = predicted_params[i]
                
                # 应用合理的范围限制
                if param_name == 'nFeatures':
                    optimized_params[param_name] = int(max(200, min(2000, val)))
                elif param_name == 'scaleFactor':
                    optimized_params[param_name] = max(1.01, min(2.0, val))
                elif param_name == 'nLevels':
                    optimized_params[param_name] = int(max(3, min(12, val)))
                elif param_name in ['thRefRatio', 'KeyFrameLinearity']:
                    optimized_params[param_name] = max(0.1, min(1.0, val))
                elif param_name == 'KeyFrameAngularity':
                    optimized_params[param_name] = max(0.1, min(2.0, val))
                elif param_name == 'minScore':
                    optimized_params[param_name] = max(0.01, min(0.5, val))
                elif param_name == 'nSimilarityGroups':
                    optimized_params[param_name] = int(max(1, min(8, val)))
            
            self.logger.debug(f"🤖 AI-optimized parameters: {optimized_params}")
            return optimized_params
            
        except Exception as e:
            self.logger.error(f"Trained model optimization error: {e}")
            return self._optimize_with_heuristics(env_features)
    
    def _optimize_with_heuristics(self, env_features: Dict) -> Dict:
        """使用启发式方法进行参数优化"""
        try:
            env_type = self.classify_environment_type(env_features)
            base_params = self.heuristic_params[env_type].copy()
            
            # 应用简单的启发式调整
            adjustments = {}
            
            # 基于亮度调整
            if env_features['brightness'] < 0.3:  # 低光照
                adjustments['nFeatures'] = base_params[0] * 0.8
                adjustments['thRefRatio'] = base_params[3] * 1.1
            elif env_features['brightness'] > 0.8:  # 强光
                adjustments['nFeatures'] = base_params[0] * 1.1
                
            # 基于纹理密度调整
            if env_features['texture_density'] > 0.5:  # 高纹理
                adjustments['nFeatures'] = base_params[0] * 1.2
                adjustments['scaleFactor'] = base_params[1] * 1.05
            elif env_features['texture_density'] < 0.2:  # 低纹理
                adjustments['nFeatures'] = base_params[0] * 0.7
                
            # 基于运动调整
            if env_features['motion_magnitude'] > 0.5:  # 快速运动
                adjustments['KeyFrameAngularity'] = base_params[5] * 1.2
                adjustments['minScore'] = base_params[6] * 1.1
            
            # 构建最终参数
            param_names = ['nFeatures', 'scaleFactor', 'nLevels', 'thRefRatio', 
                          'KeyFrameLinearity', 'KeyFrameAngularity', 'minScore', 'nSimilarityGroups']
            
            optimized_params = {}
            for i, param_name in enumerate(param_names):
                if param_name in adjustments:
                    val = adjustments[param_name]
                else:
                    val = base_params[i]
                
                # 应用类型转换和范围限制
                if param_name in ['nFeatures', 'nLevels', 'nSimilarityGroups']:
                    optimized_params[param_name] = int(max(100, min(2000, val)))
                else:
                    optimized_params[param_name] = max(0.01, min(5.0, val))
            
            self.logger.debug(f"🔧 Heuristic-optimized parameters: {optimized_params}")
            return optimized_params
            
        except Exception as e:
            self.logger.error(f"Heuristic optimization error: {e}")
            return self._get_default_params()
    
    def _get_default_params(self) -> Dict:
        """返回默认SLAM参数"""
        return {
            'nFeatures': 1000,
            'scaleFactor': 1.2,
            'nLevels': 8,
            'thRefRatio': 0.75,
            'KeyFrameLinearity': 0.9,
            'KeyFrameAngularity': 0.25,
            'minScore': 0.05,
            'nSimilarityGroups': 3
        }
    
    def update_performance_feedback(self, performance_score: float):
        """更新性能反馈"""
        self.performance_history.append(performance_score)
        if len(self.performance_history) > self.max_history_length:
            self.performance_history.pop(0)
    
    def get_performance_stats(self) -> Dict[str, float]:
        """获取性能统计信息"""
        if not self.performance_history:
            return {
                'avg_performance': 0.0, 
                'recent_trend': 0.0,
                'sample_count': 0
            }
        
        avg_performance = np.mean(self.performance_history)
        
        # 计算最近趋势
        if len(self.performance_history) >= 10:
            recent_avg = np.mean(self.performance_history[-5:])
            earlier_avg = np.mean(self.performance_history[-10:-5])
            recent_trend = recent_avg - earlier_avg
        else:
            recent_trend = 0.0
        
        return {
            'avg_performance': avg_performance,
            'recent_trend': recent_trend,
            'sample_count': len(self.performance_history)
        }


class PerformanceMonitor:
    """SLAM性能监控器"""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.tracking_quality_history = []
        self.mapping_quality_history = []
        self.computation_time_history = []
        
    def evaluate_tracking_quality(self, pose_data: Dict) -> float:
        """评估跟踪质量"""
        try:
            # 基于位姿数据评估跟踪质量
            # 这里是简化版本，实际应该基于ORB-SLAM3的跟踪状态
            if 'confidence' in pose_data:
                return pose_data['confidence']
            elif 'covariance' in pose_data:
                # 基于协方差矩阵评估
                covariance = np.array(pose_data['covariance'])
                quality = 1.0 / (1.0 + np.trace(covariance))
                return min(1.0, max(0.0, quality))
            else:
                # 默认评分
                return 0.7
                
        except Exception as e:
            self.logger.error(f"Tracking quality evaluation error: {e}")
            return 0.5
    
    def evaluate_mapping_quality(self, map_data: Dict) -> float:
        """评估建图质量"""
        try:
            # 基于地图点数量和分布评估
            if 'num_map_points' in map_data:
                num_points = map_data['num_map_points']
                # 正常化点云数量评分
                quality = min(1.0, num_points / 10000.0)  # 假设10000个点为满分
                return quality
            else:
                return 0.6
                
        except Exception as e:
            self.logger.error(f"Mapping quality evaluation error: {e}")
            return 0.5
    
    def record_computation_time(self, time_ms: float):
        """记录计算时间"""
        self.computation_time_history.append(time_ms)
        if len(self.computation_time_history) > 100:
            self.computation_time_history.pop(0)
    
    def get_overall_performance(self) -> float:
        """获取综合性能评分"""
        try:
            tracking_score = np.mean(self.tracking_quality_history) if self.tracking_quality_history else 0.5
            mapping_score = np.mean(self.mapping_quality_history) if self.mapping_quality_history else 0.5
            
            # 计算效率分数（基于计算时间）
            if self.computation_time_history:
                avg_time = np.mean(self.computation_time_history)
                efficiency_score = max(0.0, 1.0 - (avg_time - 30.0) / 100.0)  # 30ms为理想时间
                efficiency_score = min(1.0, efficiency_score)
            else:
                efficiency_score = 0.5
            
            # 综合评分
            overall_score = 0.5 * tracking_score + 0.3 * mapping_score + 0.2 * efficiency_score
            return overall_score
            
        except Exception as e:
            self.logger.error(f"Performance evaluation error: {e}")
            return 0.5