import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO

class AppleDetector(Node):
    def __init__(self):
        super().__init__('apple_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        
        # 加载训练好的自定义YOLOv8苹果检测模型
        # 使用绝对路径指向工作空间根目录
        model_path = '/home/wb/ros_ws/yolov8_apple_custom.pt'
        self.get_logger().info(f'Loading custom YOLOv8 apple detection model from: {model_path}')
        
        try:
            self.model = YOLO(model_path)
            self.get_logger().info('Custom YOLOv8 apple detection model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load custom YOLOv8 apple detection model: {e}')
            self.model = None

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        if self.model is not None:
            # 使用YOLOv8模型进行检测
            results = self.model(cv_image, conf=0.5)  # 置信度阈值0.5
            
            # 在图像上绘制检测结果
            annotated_image = cv_image.copy()
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # 获取边界框坐标
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        
                        # 获取置信度
                        confidence = float(box.conf[0])
                        
                        # 获取类别
                        class_id = int(box.cls[0])
                        class_name = result.names[class_id]
                        
                        # 绘制边界框
                        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # 绘制标签
                        label = f'{class_name}: {confidence:.2f}'
                        cv2.putText(annotated_image, label, (x1, y1-10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        # 发布检测信息
                        self.get_logger().info(f'Detected {class_name} with confidence: {confidence:.2f}')
            
            # 显示结果
            cv2.imshow("Custom YOLOv8 Apple Detection", annotated_image)
        else:
            # 如果模型加载失败，使用原来的HSV方法作为备选
            self.get_logger().warn('Using HSV fallback method')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([179, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            result = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            
            # 轮廓检测
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:  # 过滤小噪声
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(result, (x, y), (x+w, y+h), (0,255,0), 2)
                    cv2.putText(result, "Apple (HSV)", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
            
            cv2.imshow("HSV Apple Detection", result)
        
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AppleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 