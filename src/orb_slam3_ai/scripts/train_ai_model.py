#!/usr/bin/env python3
"""
ORB-SLAM3 AI模型训练脚本
用于训练自定义的SLAM参数优化模型
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np
import cv2
import json
import os
from typing import Dict, List, Tuple
import logging
from datetime import datetime

class SLAMDataset(Dataset):
    """SLAM训练数据集"""
    
    def __init__(self, data_dir: str, transform=None):
        self.data_dir = data_dir
        self.transform = transform
        self.samples = self._load_samples()
        
    def _load_samples(self) -> List[Dict]:
        """加载训练样本"""
        samples = []
        data_file = os.path.join(self.data_dir, 'training_data.json')
        
        if os.path.exists(data_file):
            with open(data_file, 'r') as f:
                samples = json.load(f)
        
        return samples
    
    def __len__(self):
        return len(self.samples)
    
    def __getitem__(self, idx):
        sample = self.samples[idx]
        
        # 加载图像
        image_path = os.path.join(self.data_dir, sample['image_path'])
        image = cv2.imread(image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        if self.transform:
            image = self.transform(image)
        
        # 环境特征
        env_features = torch.tensor(sample['env_features'], dtype=torch.float32)
        
        # 目标参数
        target_params = torch.tensor(sample['optimal_params'], dtype=torch.float32)
        
        return {
            'image': image,
            'env_features': env_features,
            'target_params': target_params,
            'performance_score': sample['performance_score']
        }

class SLAMParameterNetwork(nn.Module):
    """SLAM参数预测网络"""
    
    def __init__(self, feature_dim=512, env_feature_dim=8, output_dim=8):
        super().__init__()
        
        # 图像特征提取器（使用预训练ResNet18）
        import torchvision.models as models
        self.backbone = models.resnet18(pretrained=True)
        self.backbone.fc = nn.Identity()
        
        # 冻结backbone的前几层
        for param in list(self.backbone.parameters())[:-20]:
            param.requires_grad = False
        
        # 参数预测网络
        self.param_predictor = nn.Sequential(
            nn.Linear(feature_dim + env_feature_dim, 512),
            nn.ReLU(),
            nn.BatchNorm1d(512),
            nn.Dropout(0.3),
            
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.BatchNorm1d(256),
            nn.Dropout(0.2),
            
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.BatchNorm1d(128),
            nn.Dropout(0.1),
            
            nn.Linear(128, output_dim),
            nn.Tanh()  # 输出范围 [-1, 1]
        )
    
    def forward(self, image, env_features):
        # 提取图像特征
        img_features = self.backbone(image)
        
        # 组合特征
        combined_features = torch.cat([img_features, env_features], dim=1)
        
        # 预测参数调整
        param_adjustments = self.param_predictor(combined_features)
        
        return param_adjustments

class SLAMTrainer:
    """SLAM AI模型训练器"""
    
    def __init__(self, model_save_dir: str):
        self.model_save_dir = model_save_dir
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # 创建模型
        self.model = SLAMParameterNetwork().to(self.device)
        
        # 优化器
        self.optimizer = optim.AdamW(
            self.model.parameters(),
            lr=1e-4,
            weight_decay=1e-5
        )
        
        # 学习率调度器
        self.scheduler = optim.lr_scheduler.ReduceLROnPlateau(
            self.optimizer, mode='min', patience=10, factor=0.5
        )
        
        # 损失函数
        self.criterion = nn.MSELoss()
        
        # 日志
        self.logger = logging.getLogger(__name__)
        
    def train_epoch(self, dataloader: DataLoader) -> float:
        """训练一个epoch"""
        self.model.train()
        total_loss = 0.0
        
        for batch_idx, batch in enumerate(dataloader):
            # 移动数据到设备
            images = batch['image'].to(self.device)
            env_features = batch['env_features'].to(self.device)
            target_params = batch['target_params'].to(self.device)
            
            # 前向传播
            self.optimizer.zero_grad()
            predicted_params = self.model(images, env_features)
            
            # 计算损失
            loss = self.criterion(predicted_params, target_params)
            
            # 反向传播
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
            self.optimizer.step()
            
            total_loss += loss.item()
            
            if batch_idx % 10 == 0:
                self.logger.info(f'Batch {batch_idx}, Loss: {loss.item():.6f}')
        
        return total_loss / len(dataloader)
    
    def validate(self, dataloader: DataLoader) -> float:
        """验证模型"""
        self.model.eval()
        total_loss = 0.0
        
        with torch.no_grad():
            for batch in dataloader:
                images = batch['image'].to(self.device)
                env_features = batch['env_features'].to(self.device)
                target_params = batch['target_params'].to(self.device)
                
                predicted_params = self.model(images, env_features)
                loss = self.criterion(predicted_params, target_params)
                total_loss += loss.item()
        
        return total_loss / len(dataloader)
    
    def train(self, train_loader: DataLoader, val_loader: DataLoader, 
              num_epochs: int = 100):
        """完整训练流程"""
        
        best_val_loss = float('inf')
        patience_counter = 0
        max_patience = 20
        
        for epoch in range(num_epochs):
            self.logger.info(f'Epoch {epoch+1}/{num_epochs}')
            
            # 训练
            train_loss = self.train_epoch(train_loader)
            
            # 验证
            val_loss = self.validate(val_loader)
            
            # 学习率调整
            self.scheduler.step(val_loss)
            
            self.logger.info(f'Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}')
            
            # 保存最佳模型
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                patience_counter = 0
                self.save_model(f'best_model_epoch_{epoch+1}.pth')
            else:
                patience_counter += 1
            
            # 早停
            if patience_counter >= max_patience:
                self.logger.info(f'Early stopping at epoch {epoch+1}')
                break
    
    def save_model(self, filename: str):
        """保存模型"""
        model_path = os.path.join(self.model_save_dir, filename)
        os.makedirs(self.model_save_dir, exist_ok=True)
        
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'model_config': {
                'feature_dim': 512,
                'env_feature_dim': 8,
                'output_dim': 8
            }
        }, model_path)
        
        self.logger.info(f'Model saved to {model_path}')

def collect_training_data():
    """数据收集指南"""
    guide = """
    📊 ORB-SLAM3 AI模型训练数据收集指南
    
    1. 数据收集环境
    ================
    - 多种环境: 室内、室外、不同光照条件
    - 多种纹理: 高纹理、低纹理、重复纹理
    - 多种运动: 慢速、快速、静止、旋转
    
    2. 数据格式
    ===========
    training_data/
    ├── images/           # RGB图像
    ├── depth/            # 深度图像
    ├── ground_truth/     # 真实轨迹
    ├── slam_results/     # SLAM输出结果
    └── training_data.json # 训练标签
    
    3. 标签生成
    ===========
    对每个场景：
    - 运行原版ORB-SLAM3，记录性能
    - 手动调优参数，获得最佳性能
    - 记录最优参数组合
    - 计算性能评分(轨迹精度、地图质量)
    
    4. 数据样本格式
    ===============
    {
        "image_path": "images/scene_001.jpg",
        "depth_path": "depth/scene_001.png", 
        "env_features": [brightness, contrast, texture_density, ...],
        "optimal_params": [nFeatures, scaleFactor, nLevels, ...],
        "performance_score": 0.95,
        "scene_type": "indoor_low_light",
        "motion_type": "slow_translation"
    }
    
    5. 训练建议
    ===========
    - 数据集大小: 10,000+ 样本
    - 训练/验证/测试 = 7:2:1
    - 使用数据增强: 光照变化、噪声添加
    - 定期在真实数据上验证
    """
    return guide

if __name__ == '__main__':
    # 设置日志
    logging.basicConfig(level=logging.INFO)
    
    # 打印数据收集指南
    print(collect_training_data())
    
    # 示例训练代码
    print("\\n🚀 开始模型训练...")
    
    # 数据路径
    data_dir = '/home/wb/ros_ws/slam_training_data'
    model_save_dir = '/home/wb/ros_ws/trained_models'
    
    if os.path.exists(data_dir):
        # 创建数据加载器
        from torchvision import transforms
        
        transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                               std=[0.229, 0.224, 0.225])
        ])
        
        # 数据集
        full_dataset = SLAMDataset(data_dir, transform=transform)
        
        # 划分数据集
        train_size = int(0.8 * len(full_dataset))
        val_size = len(full_dataset) - train_size
        train_dataset, val_dataset = torch.utils.data.random_split(
            full_dataset, [train_size, val_size]
        )
        
        # 数据加载器
        train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
        val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)
        
        # 训练器
        trainer = SLAMTrainer(model_save_dir)
        
        # 开始训练
        trainer.train(train_loader, val_loader, num_epochs=100)
        
        print("✅ 训练完成!")
    else:
        print(f"❌ 训练数据目录不存在: {data_dir}")
        print("请先收集训练数据，参考上面的数据收集指南")