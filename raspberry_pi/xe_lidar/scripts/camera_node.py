#!/usr/bin/env python3
"""
ROS2 Camera Node sử dụng OpenCV (KHÔNG CẦN cv_bridge)
Publish ảnh từ USB camera lên topic /camera/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Parameters
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_link_optical')
        
        video_device = self.get_parameter('video_device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Khởi tạo camera
        self.cap = cv2.VideoCapture(video_device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Không thể mở camera tại {video_device}')
            sys.exit(1)
        
        # Cấu hình camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        self.get_logger().info(f'Camera đã mở tại {video_device} ({width}x{height} @ {fps}fps)')
        
        # Publisher (publish vào /camera/image_raw để tương thích với code hiện tại)
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Timer để publish ảnh
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Camera Node đã khởi động! (KHÔNG CẦN cv_bridge)')
    
    def cv2_to_imgmsg(self, cv_image, encoding="bgr8"):
        """
        Convert OpenCV image (numpy array) sang ROS2 Image message
        KHÔNG CẦN cv_bridge!
        """
        img_msg = Image()
        img_msg.height, img_msg.width = cv_image.shape[:2]
        
        if encoding == "bgr8":
            img_msg.encoding = "bgr8"
            img_msg.is_bigendian = 0
            img_msg.step = img_msg.width * 3  # 3 bytes per pixel (BGR)
            img_msg.data = cv_image.tobytes()
        elif encoding == "rgb8":
            img_msg.encoding = "rgb8"
            img_msg.is_bigendian = 0
            img_msg.step = img_msg.width * 3
            img_msg.data = cv_image.tobytes()
        else:
            raise ValueError(f"Encoding {encoding} chưa được hỗ trợ")
        
        return img_msg
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                # Convert OpenCV image sang ROS2 Image message (KHÔNG CẦN cv_bridge)
                ros_image = self.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = self.frame_id
                
                # Publish
                self.publisher.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f'Lỗi convert ảnh: {str(e)}')
        else:
            self.get_logger().warn('Không đọc được frame từ camera')
    
    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'Lỗi trong node: {str(e)}')
    finally:
        if node:
            try:
                node.destroy_node()
            except:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()

