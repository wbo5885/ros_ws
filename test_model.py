#!/usr/bin/env python3

import os
import cv2
import numpy as np
from ultralytics import YOLO

def test_custom_model():
    """测试自定义YOLOv8苹果检测模型"""
    
    # 模型路径
    model_path = 'yolov8_apple_custom.pt'
    
    print(f"正在加载模型: {model_path}")
    
    try:
        # 加载模型
        model = YOLO(model_path)
        print("✅ 模型加载成功!")
        
        # 获取模型信息
        print(f"模型类别数量: {len(model.names)}")
        print(f"模型类别名称: {model.names}")
        
        # 创建一个测试图像（红色圆形模拟苹果）
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 绘制红色圆形
        cv2.circle(test_image, (320, 240), 80, (0, 0, 255), -1)
        
        print("正在测试模型推理...")
        
        # 进行推理
        results = model(test_image, conf=0.5)
        
        print(f"检测到 {len(results)} 个结果")
        
        for i, result in enumerate(results):
            boxes = result.boxes
            if boxes is not None:
                print(f"结果 {i+1}: 检测到 {len(boxes)} 个目标")
                for j, box in enumerate(boxes):
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = result.names[class_id]
                    print(f"  目标 {j+1}: {class_name}, 置信度: {confidence:.3f}")
            else:
                print(f"结果 {i+1}: 未检测到目标")
        
        print("✅ 模型测试完成!")
        return True
        
    except Exception as e:
        print(f"❌ 模型测试失败: {e}")
        return False

if __name__ == "__main__":
    test_custom_model() 