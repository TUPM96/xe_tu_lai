#!/usr/bin/env python3
"""
ROS2 Camera Node sử dụng OpenCV (thay thế v4l2_camera)
Publish ảnh từ USB camera lên topic /camera/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
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
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Timer để publish ảnh
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Camera Node đã khởi động!')
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                # Convert OpenCV image sang ROS2 Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
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

