#!/usr/bin/env python3
"""
改进的ORB-SLAM3 AI模型训练脚本
使用数据归一化和改进的损失函数
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np
import cv2
import json
import os
from typing import Dict, List
import logging
from datetime import datetime

class ImprovedSLAMDataset(Dataset):
    """改进的SLAM数据集，带数据归一化"""
    
    def __init__(self, data_dir: str):
        self.data_dir = data_dir
        self.samples = self._load_samples()
        self.param_stats = self._calculate_param_stats()
        
    def _load_samples(self) -> List[Dict]:
        """加载训练样本"""
        samples = []
        data_file = os.path.join(self.data_dir, 'training_data.json')
        
        if os.path.exists(data_file):
            with open(data_file, 'r') as f:
                samples = json.load(f)
        
        return samples
    
    def _calculate_param_stats(self):
        """计算参数统计信息用于归一化"""
        if not self.samples:
            return None
            
        params = [sample['optimal_params'] for sample in self.samples]
        params = np.array(params)
        
        return {
            'mean': np.mean(params, axis=0),
            'std': np.std(params, axis=0) + 1e-8  # 避免除零
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
    
    def __len__(self):
        return len(self.samples)
    
    def __getitem__(self, idx):
        sample = self.samples[idx]
        
        # 加载图像
        image_path = os.path.join(self.data_dir, sample['image_path'])
        image = cv2.imread(image_path)
        if image is None:
            # 创建虚拟图像作为备用
            image = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        else:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # 图像预处理
        image = cv2.resize(image, (64, 64))
        image = image.astype(np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))  # CHW格式
        
        # 环境特征归一化
        env_features = np.array(sample['env_features'], dtype=np.float32)
        env_features = np.clip(env_features, 0, 10)  # 限制范围
        
        # 目标参数归一化
        target_params = np.array(sample['optimal_params'], dtype=np.float32)
        target_params = self.normalize_params(target_params)
        
        return {
            'image': torch.tensor(image),
            'env_features': torch.tensor(env_features),
            'target_params': torch.tensor(target_params),
            'performance_score': sample['performance_score']
        }

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
            # 不使用激活函数，让模型学习任意范围的输出
        )
    
    def forward(self, image, env_features):
        # 提取图像特征
        img_features = self.feature_extractor(image)
        
        # 组合特征
        combined_features = torch.cat([img_features, env_features], dim=1)
        
        # 预测参数调整
        param_adjustments = self.param_predictor(combined_features)
        
        return param_adjustments

def improved_train():
    """改进的训练函数"""
    print("🚀 开始改进的AI模型训练...")
    
    # 设置
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"🖥️ 使用设备: {device}")
    
    data_dir = '/home/wb/ros_ws/slam_training_data'
    model_save_dir = '/home/wb/ros_ws/src/orb_slam3_ai/models/trained'
    
    # 创建保存目录
    os.makedirs(model_save_dir, exist_ok=True)
    
    # 数据集
    dataset = ImprovedSLAMDataset(data_dir)
    print(f"📊 数据集大小: {len(dataset)} 样本")
    
    if len(dataset) == 0:
        print("❌ 没有找到训练数据")
        return
    
    # 打印参数统计信息
    if dataset.param_stats:
        print("📈 参数统计信息:")
        print(f"  均值: {dataset.param_stats['mean']}")
        print(f"  标准差: {dataset.param_stats['std']}")
    
    # 数据加载器
    dataloader = DataLoader(dataset, batch_size=4, shuffle=True)
    
    # 模型
    model = ImprovedSLAMNetwork().to(device)
    optimizer = optim.AdamW(model.parameters(), lr=0.001, weight_decay=1e-4)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', patience=10, factor=0.5)
    
    # 使用Huber Loss，对异常值更鲁棒
    criterion = nn.SmoothL1Loss()
    
    print("🏋️ 开始训练...")
    
    # 训练循环
    num_epochs = 100
    best_loss = float('inf')
    
    for epoch in range(num_epochs):
        model.train()
        total_loss = 0.0
        num_batches = 0
        
        for batch in dataloader:
            images = batch['image'].to(device)
            env_features = batch['env_features'].to(device)
            target_params = batch['target_params'].to(device)
            
            # 前向传播
            optimizer.zero_grad()
            predicted_params = model(images, env_features)
            
            # 计算损失
            loss = criterion(predicted_params, target_params)
            
            # 反向传播
            loss.backward()
            # 梯度裁剪
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            
            total_loss += loss.item()
            num_batches += 1
        
        avg_loss = total_loss / num_batches if num_batches > 0 else 0
        scheduler.step(avg_loss)
        
        # 保存最佳模型
        if avg_loss < best_loss:
            best_loss = avg_loss
            best_model_path = os.path.join(model_save_dir, 'best_slam_model.pth')
            torch.save({
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'model_config': {
                    'env_feature_dim': 8,
                    'output_dim': 8,
                    'model_type': 'ImprovedSLAMNetwork'
                },
                'param_stats': dataset.param_stats,
                'training_info': {
                    'epoch': epoch + 1,
                    'best_loss': best_loss,
                    'dataset_size': len(dataset),
                    'timestamp': datetime.now().isoformat()
                }
            }, best_model_path)
        
        if (epoch + 1) % 20 == 0:
            current_lr = optimizer.param_groups[0]['lr']
            print(f"Epoch {epoch+1}/{num_epochs}, Loss: {avg_loss:.6f}, LR: {current_lr:.2e}, Best: {best_loss:.6f}")
    
    print(f"✅ 训练完成!")
    print(f"📁 最佳模型已保存到: {best_model_path}")
    print(f"📊 最佳损失: {best_loss:.6f}")
    
    # 测试模型
    print("\n🧪 测试训练好的模型...")
    model.eval()
    test_losses = []
    
    with torch.no_grad():
        for i in range(min(5, len(dataset))):  # 测试前5个样本
            test_sample = dataset[i]
            test_image = test_sample['image'].unsqueeze(0).to(device)
            test_env = test_sample['env_features'].unsqueeze(0).to(device)
            test_target = test_sample['target_params'].unsqueeze(0).to(device)
            
            prediction = model(test_image, test_env)
            test_loss = criterion(prediction, test_target).item()
            test_losses.append(test_loss)
            
            # 反归一化进行可视化
            pred_denorm = dataset.denormalize_params(prediction.squeeze().cpu().numpy())
            target_denorm = dataset.denormalize_params(test_target.squeeze().cpu().numpy())
            
            print(f"\n样本 {i+1}:")
            print(f"  目标参数: {target_denorm}")
            print(f"  预测参数: {pred_denorm}")
            print(f"  相对误差: {np.mean(np.abs((pred_denorm - target_denorm) / (target_denorm + 1e-8))) * 100:.2f}%")
    
    print(f"\n📊 测试统计:")
    print(f"  平均测试损失: {np.mean(test_losses):.6f}")
    print(f"  测试损失标准差: {np.std(test_losses):.6f}")

if __name__ == '__main__':
    improved_train()