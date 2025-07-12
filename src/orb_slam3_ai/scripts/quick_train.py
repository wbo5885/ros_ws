#!/usr/bin/env python3
"""
快速ORB-SLAM3 AI模型训练脚本
用于小数据集的快速原型训练
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

class QuickSLAMDataset(Dataset):
    """快速SLAM数据集"""
    
    def __init__(self, data_dir: str):
        self.data_dir = data_dir
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
        if image is None:
            # 创建虚拟图像作为备用
            image = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        else:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # 简单的图像预处理
        image = cv2.resize(image, (64, 64))  # 更小的尺寸用于快速训练
        image = image.astype(np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))  # CHW格式
        
        # 环境特征
        env_features = np.array(sample['env_features'], dtype=np.float32)
        
        # 目标参数
        target_params = np.array(sample['optimal_params'], dtype=np.float32)
        
        return {
            'image': torch.tensor(image),
            'env_features': torch.tensor(env_features),
            'target_params': torch.tensor(target_params),
            'performance_score': sample['performance_score']
        }

class SimpleSLAMNetwork(nn.Module):
    """简化的SLAM参数预测网络"""
    
    def __init__(self, env_feature_dim=8, output_dim=8):
        super().__init__()
        
        # 简单的CNN特征提取器
        self.feature_extractor = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((4, 4)),
            nn.Flatten(),
            nn.Linear(64 * 4 * 4, 128),
            nn.ReLU()
        )
        
        # 参数预测网络
        self.param_predictor = nn.Sequential(
            nn.Linear(128 + env_feature_dim, 64),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, output_dim),
            nn.Tanh()  # 输出范围 [-1, 1]
        )
    
    def forward(self, image, env_features):
        # 提取图像特征
        img_features = self.feature_extractor(image)
        
        # 组合特征
        combined_features = torch.cat([img_features, env_features], dim=1)
        
        # 预测参数调整
        param_adjustments = self.param_predictor(combined_features)
        
        return param_adjustments

def quick_train():
    """快速训练函数"""
    print("🚀 开始快速AI模型训练...")
    
    # 设置
    device = torch.device('cpu')  # 使用CPU避免GPU设置问题
    data_dir = '/home/wb/ros_ws/slam_training_data'
    model_save_dir = '/home/wb/ros_ws/src/orb_slam3_ai/models/trained'
    
    # 创建保存目录
    os.makedirs(model_save_dir, exist_ok=True)
    
    # 数据集
    dataset = QuickSLAMDataset(data_dir)
    print(f"📊 数据集大小: {len(dataset)} 样本")
    
    if len(dataset) == 0:
        print("❌ 没有找到训练数据")
        return
    
    # 数据加载器
    dataloader = DataLoader(dataset, batch_size=4, shuffle=True)
    
    # 模型
    model = SimpleSLAMNetwork().to(device)
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    criterion = nn.MSELoss()
    
    print("🏋️ 开始训练...")
    
    # 训练循环
    num_epochs = 50  # 快速训练，较少epoch
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
            optimizer.step()
            
            total_loss += loss.item()
            num_batches += 1
        
        avg_loss = total_loss / num_batches if num_batches > 0 else 0
        
        if (epoch + 1) % 10 == 0:
            print(f"Epoch {epoch+1}/{num_epochs}, Average Loss: {avg_loss:.6f}")
    
    # 保存模型
    model_path = os.path.join(model_save_dir, 'quick_slam_model.pth')
    torch.save({
        'model_state_dict': model.state_dict(),
        'model_config': {
            'env_feature_dim': 8,
            'output_dim': 8,
            'model_type': 'SimpleSLAMNetwork'
        },
        'training_info': {
            'epochs': num_epochs,
            'final_loss': avg_loss,
            'dataset_size': len(dataset),
            'timestamp': datetime.now().isoformat()
        }
    }, model_path)
    
    print(f"✅ 训练完成!")
    print(f"📁 模型已保存到: {model_path}")
    print(f"📊 最终损失: {avg_loss:.6f}")
    
    # 测试模型
    print("\n🧪 测试训练好的模型...")
    model.eval()
    with torch.no_grad():
        test_sample = dataset[0]
        test_image = test_sample['image'].unsqueeze(0).to(device)
        test_env = test_sample['env_features'].unsqueeze(0).to(device)
        test_target = test_sample['target_params']
        
        prediction = model(test_image, test_env)
        
        print(f"目标参数: {test_target.numpy()}")
        print(f"预测参数: {prediction.squeeze().cpu().numpy()}")
        print(f"预测误差: {torch.mean(torch.abs(prediction.squeeze().cpu() - test_target)).item():.6f}")

if __name__ == '__main__':
    quick_train()