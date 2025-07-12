#!/usr/bin/env python3
"""
ORB-SLAM3 AI训练数据集下载脚本
从开源网站下载标准SLAM数据集用于训练
"""

import os
import sys
import wget
import tarfile
import zipfile
import cv2
import numpy as np
import json
from pathlib import Path
import shutil
from typing import Dict, List, Tuple
import argparse

class DatasetDownloader:
    """SLAM数据集下载器"""
    
    def __init__(self, base_dir="/home/wb/ros_ws/slam_training_data"):
        self.base_dir = Path(base_dir)
        self.datasets_dir = self.base_dir / "raw_datasets"
        self.processed_dir = self.base_dir / "processed"
        
        # 创建目录
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.datasets_dir.mkdir(parents=True, exist_ok=True)
        self.processed_dir.mkdir(parents=True, exist_ok=True)
        
        # TUM RGB-D数据集URLs
        self.tum_datasets = {
            'freiburg1_xyz': 'https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz',
            'freiburg1_desk': 'https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz',
            'freiburg2_xyz': 'https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_xyz.tgz',
            'freiburg2_desk': 'https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_desk.tgz',
            'freiburg3_office': 'https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz'
        }
        
        # EuRoC MAV数据集URLs (选择部分有代表性的序列)
        self.euroc_datasets = {
            'MH_01_easy': 'http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip',
            'MH_03_medium': 'http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_03_medium/MH_03_medium.zip',
            'V1_01_easy': 'http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip',
            'V2_01_easy': 'http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room2/V2_01_easy/V2_01_easy.zip'
        }
        
        print(f"📁 数据集将下载到: {self.base_dir}")
    
    def download_file(self, url: str, filename: str) -> bool:
        """下载文件"""
        filepath = self.datasets_dir / filename
        
        if filepath.exists():
            print(f"✅ 文件已存在: {filename}")
            return True
            
        try:
            print(f"🔄 下载中: {filename}")
            print(f"   URL: {url}")
            
            # 使用wget下载文件
            wget.download(url, str(filepath))
            print(f"\\n✅ 下载完成: {filename}")
            return True
            
        except Exception as e:
            print(f"❌ 下载失败: {filename} - {e}")
            if filepath.exists():
                filepath.unlink()
            return False
    
    def extract_archive(self, filepath: Path, extract_dir: Path) -> bool:
        """解压文件"""
        try:
            print(f"📦 解压中: {filepath.name}")
            
            if filepath.suffix == '.tgz' or filepath.name.endswith('.tar.gz'):
                with tarfile.open(filepath, 'r:gz') as tar:
                    tar.extractall(extract_dir)
            elif filepath.suffix == '.zip':
                with zipfile.ZipFile(filepath, 'r') as zip_ref:
                    zip_ref.extractall(extract_dir)
            else:
                print(f"❌ 不支持的文件格式: {filepath.suffix}")
                return False
                
            print(f"✅ 解压完成: {filepath.name}")
            return True
            
        except Exception as e:
            print(f"❌ 解压失败: {filepath.name} - {e}")
            return False
    
    def download_tum_datasets(self, datasets: List[str] = None):
        """下载TUM RGB-D数据集"""
        print("\\n🤖 开始下载TUM RGB-D数据集...")
        
        if datasets is None:
            datasets = list(self.tum_datasets.keys())
        
        tum_dir = self.datasets_dir / "TUM_RGB-D"
        tum_dir.mkdir(exist_ok=True)
        
        for dataset_name in datasets:
            if dataset_name not in self.tum_datasets:
                print(f"❌ 未知数据集: {dataset_name}")
                continue
                
            url = self.tum_datasets[dataset_name]
            filename = f"{dataset_name}.tgz"
            
            # 下载
            if self.download_file(url, filename):
                # 解压
                filepath = self.datasets_dir / filename
                self.extract_archive(filepath, tum_dir)
    
    def download_euroc_datasets(self, datasets: List[str] = None):
        """下载EuRoC MAV数据集"""
        print("\\n🚁 开始下载EuRoC MAV数据集...")
        
        if datasets is None:
            datasets = list(self.euroc_datasets.keys())
        
        euroc_dir = self.datasets_dir / "EuRoC_MAV"
        euroc_dir.mkdir(exist_ok=True)
        
        for dataset_name in datasets:
            if dataset_name not in self.euroc_datasets:
                print(f"❌ 未知数据集: {dataset_name}")
                continue
                
            url = self.euroc_datasets[dataset_name]
            filename = f"{dataset_name}.zip"
            
            # 下载
            if self.download_file(url, filename):
                # 解压
                filepath = self.datasets_dir / filename
                self.extract_archive(filepath, euroc_dir)
    
    def process_tum_dataset(self, dataset_name: str) -> List[Dict]:
        """处理TUM数据集格式"""
        print(f"🔄 处理TUM数据集: {dataset_name}")
        
        dataset_dir = None
        for d in (self.datasets_dir / "TUM_RGB-D").iterdir():
            if d.is_dir() and dataset_name in d.name:
                dataset_dir = d
                break
        
        if not dataset_dir:
            print(f"❌ 找不到数据集目录: {dataset_name}")
            return []
        
        print(f"📁 处理目录: {dataset_dir}")
        
        # 读取关联文件
        associations_file = dataset_dir / "associations.txt"
        if not associations_file.exists():
            # 生成关联文件
            self.generate_tum_associations(dataset_dir)
        
        samples = []
        rgb_dir = dataset_dir / "rgb"
        depth_dir = dataset_dir / "depth"
        
        if not rgb_dir.exists() or not depth_dir.exists():
            print(f"❌ RGB或深度目录不存在")
            return []
        
        # 读取前100个样本用于快速测试
        rgb_files = sorted(list(rgb_dir.glob("*.png")))[:100]
        
        for i, rgb_file in enumerate(rgb_files):
            # 查找对应的深度图像
            timestamp = rgb_file.stem
            depth_file = depth_dir / f"{timestamp}.png"
            
            if not depth_file.exists():
                continue
            
            # 分析图像特征
            rgb_img = cv2.imread(str(rgb_file))
            if rgb_img is None:
                continue
                
            env_features = self.analyze_image_features(rgb_img)
            
            # 根据数据集特性设置环境类型
            env_type = self.classify_tum_environment(dataset_name, i)
            optimal_params = self.get_optimal_params_for_env(env_type)
            
            sample = {
                "dataset": "TUM_RGB-D",
                "sequence": dataset_name,
                "image_path": f"TUM_RGB-D/{dataset_dir.name}/rgb/{rgb_file.name}",
                "depth_path": f"TUM_RGB-D/{dataset_dir.name}/depth/{depth_file.name}",
                "env_features": env_features,
                "optimal_params": optimal_params,
                "performance_score": 0.85 + np.random.random() * 0.1,  # 模拟性能分数
                "scene_type": env_type,
                "motion_type": "mixed"
            }
            
            samples.append(sample)
        
        print(f"✅ 处理完成: {len(samples)} 个样本")
        return samples
    
    def analyze_image_features(self, image: np.ndarray) -> List[float]:
        """分析图像特征"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 亮度
        brightness = np.mean(gray) / 255.0
        
        # 对比度
        contrast = np.std(gray) / 255.0
        
        # 纹理密度 (使用Sobel边缘检测)
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        edge_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        texture_density = np.mean(edge_magnitude) / 255.0
        
        # ORB特征点数量
        orb = cv2.ORB_create()
        keypoints = orb.detect(gray, None)
        feature_count = len(keypoints) / 1000.0  # 归一化
        
        # 模拟运动数据
        motion_magnitude = np.random.random() * 0.5
        angular_velocity = np.random.random() * 0.3
        
        # 边缘密度
        edges = cv2.Canny(gray, 50, 150)
        edge_density = np.sum(edges > 0) / (edges.shape[0] * edges.shape[1])
        
        # 稳定性评分 (基于图像质量)
        stability = min(1.0, contrast * 2.0)
        
        return [
            brightness,
            contrast, 
            texture_density,
            motion_magnitude,
            angular_velocity,
            edge_density,
            feature_count,
            stability
        ]
    
    def classify_tum_environment(self, dataset_name: str, frame_idx: int) -> str:
        """根据TUM数据集名称和帧索引分类环境"""
        if 'desk' in dataset_name:
            return 'high_texture'
        elif 'xyz' in dataset_name:
            return 'normal'
        elif 'office' in dataset_name:
            return 'low_light'
        else:
            return 'normal'
    
    def get_optimal_params_for_env(self, env_type: str) -> List[float]:
        """获取环境对应的最优SLAM参数"""
        param_configs = {
            'low_texture': [600, 1.1, 6, 0.85, 0.8, 0.2, 0.1, 2],
            'high_texture': [1200, 1.25, 8, 0.7, 0.9, 0.3, 0.05, 4],
            'low_light': [500, 1.15, 6, 0.8, 0.75, 0.25, 0.08, 3],
            'fast_motion': [1000, 1.2, 7, 0.75, 0.85, 0.35, 0.06, 3],
            'normal': [1000, 1.2, 8, 0.75, 0.9, 0.25, 0.05, 3]
        }
        
        base_params = param_configs.get(env_type, param_configs['normal'])
        
        # 添加一些随机噪声来增加数据多样性
        noisy_params = []
        for param in base_params:
            noise = np.random.normal(0, 0.05)  # 5%的噪声
            noisy_params.append(max(0.1, param * (1 + noise)))
        
        return noisy_params
    
    def generate_tum_associations(self, dataset_dir: Path):
        """为TUM数据集生成关联文件"""
        rgb_dir = dataset_dir / "rgb"
        depth_dir = dataset_dir / "depth"
        
        if not rgb_dir.exists() or not depth_dir.exists():
            return
        
        rgb_files = sorted(rgb_dir.glob("*.png"))
        depth_files = sorted(depth_dir.glob("*.png"))
        
        associations = []
        for rgb_file in rgb_files:
            # 查找最近的深度文件
            rgb_timestamp = float(rgb_file.stem)
            best_depth = None
            min_diff = float('inf')
            
            for depth_file in depth_files:
                depth_timestamp = float(depth_file.stem)
                diff = abs(rgb_timestamp - depth_timestamp)
                if diff < min_diff:
                    min_diff = diff
                    best_depth = depth_file
            
            if best_depth and min_diff < 0.02:  # 20ms内的匹配
                associations.append(f"{rgb_timestamp} rgb/{rgb_file.name} {float(best_depth.stem)} depth/{best_depth.name}")
        
        # 保存关联文件
        with open(dataset_dir / "associations.txt", 'w') as f:
            f.write("\\n".join(associations))
    
    def create_training_dataset(self):
        """创建完整的训练数据集"""
        print("\\n🎯 创建训练数据集...")
        
        all_samples = []
        
        # 处理TUM数据集
        tum_dir = self.datasets_dir / "TUM_RGB-D"
        if tum_dir.exists():
            for dataset_dir in tum_dir.iterdir():
                if dataset_dir.is_dir():
                    dataset_name = dataset_dir.name.split('_')[-1] if '_' in dataset_dir.name else dataset_dir.name
                    samples = self.process_tum_dataset(dataset_name)
                    all_samples.extend(samples)
        
        # 保存训练数据
        output_file = self.base_dir / "training_data.json"
        with open(output_file, 'w') as f:
            json.dump(all_samples, f, indent=2)
        
        print(f"✅ 训练数据集创建完成!")
        print(f"   总样本数: {len(all_samples)}")
        print(f"   数据文件: {output_file}")
        
        # 创建数据集统计
        self.create_dataset_statistics(all_samples)
        
        return all_samples
    
    def create_dataset_statistics(self, samples: List[Dict]):
        """创建数据集统计信息"""
        stats = {
            "total_samples": len(samples),
            "datasets": {},
            "scene_types": {},
            "avg_features": {
                "brightness": 0,
                "contrast": 0,
                "texture_density": 0
            }
        }
        
        # 统计不同数据集
        for sample in samples:
            dataset = sample["dataset"]
            scene_type = sample["scene_type"]
            
            stats["datasets"][dataset] = stats["datasets"].get(dataset, 0) + 1
            stats["scene_types"][scene_type] = stats["scene_types"].get(scene_type, 0) + 1
            
            # 累计特征值
            features = sample["env_features"]
            stats["avg_features"]["brightness"] += features[0]
            stats["avg_features"]["contrast"] += features[1] 
            stats["avg_features"]["texture_density"] += features[2]
        
        # 计算平均值
        for key in stats["avg_features"]:
            stats["avg_features"][key] /= len(samples)
        
        # 保存统计信息
        stats_file = self.base_dir / "dataset_statistics.json"
        with open(stats_file, 'w') as f:
            json.dump(stats, f, indent=2)
        
        print(f"📊 数据集统计:")
        print(f"   总样本数: {stats['total_samples']}")
        print(f"   数据集分布: {stats['datasets']}")
        print(f"   场景类型分布: {stats['scene_types']}")

def main():
    parser = argparse.ArgumentParser(description='下载SLAM训练数据集')
    parser.add_argument('--base-dir', default='/home/wb/ros_ws/slam_training_data',
                        help='数据集保存目录')
    parser.add_argument('--tum-only', action='store_true',
                        help='只下载TUM数据集')
    parser.add_argument('--euroc-only', action='store_true', 
                        help='只下载EuRoC数据集')
    parser.add_argument('--quick', action='store_true',
                        help='快速模式，只下载部分数据集')
    
    args = parser.parse_args()
    
    # 创建下载器
    downloader = DatasetDownloader(args.base_dir)
    
    print("🚀 开始下载SLAM训练数据集...")
    print("="*50)
    
    try:
        if args.quick:
            # 快速模式：只下载小数据集
            print("⚡ 快速模式：下载精选数据集")
            downloader.download_tum_datasets(['freiburg1_xyz', 'freiburg1_desk'])
        elif args.tum_only:
            # 只下载TUM数据集
            downloader.download_tum_datasets()
        elif args.euroc_only:
            # 只下载EuRoC数据集
            downloader.download_euroc_datasets()
        else:
            # 下载所有数据集
            downloader.download_tum_datasets(['freiburg1_xyz', 'freiburg1_desk', 'freiburg2_xyz'])
            # downloader.download_euroc_datasets(['MH_01_easy'])  # EuRoC数据集较大，先注释
        
        # 处理数据集
        print("\\n" + "="*50)
        training_samples = downloader.create_training_dataset()
        
        print("\\n🎉 数据集下载和处理完成!")
        print(f"📁 数据位置: {downloader.base_dir}")
        print(f"📊 训练样本: {len(training_samples)}")
        
        # 显示使用说明
        print("\\n📖 使用说明:")
        print(f"1. 训练数据: {downloader.base_dir}/training_data.json")
        print(f"2. 原始数据: {downloader.base_dir}/raw_datasets/")
        print(f"3. 开始训练: python3 src/orb_slam3_ai/scripts/train_ai_model.py")
        
    except KeyboardInterrupt:
        print("\\n⚠️ 用户中断下载")
    except Exception as e:
        print(f"\\n❌ 下载过程出错: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()