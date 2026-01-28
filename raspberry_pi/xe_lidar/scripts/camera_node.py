#!/usr/bin/env python3
"""
ROS2 Camera Node sử dụng OpenCV (KHÔNG CẦN cv_bridge)
Publish ảnh từ USB camera lên topic /camera/image_raw
Sử dụng threaded capture để giảm giật khung hình
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
import sys
import threading


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Parameters - Mặc định 640x480 (VGA)
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_link_optical')

        video_device = self.get_parameter('video_device').value
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.get_logger().info(f'Dang mo camera {video_device} voi width={width}, height={height}, fps={fps}')

        # Lấy index từ video device (vd: /dev/video0 -> 0)
        try:
            cam_index = int(video_device.replace('/dev/video', ''))
        except:
            cam_index = 0

        # Mở camera với V4L2 backend
        self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().warn(f'V4L2 khong mo duoc, thu path truc tiep...')
            self.cap = cv2.VideoCapture(video_device)

        if not self.cap.isOpened():
            self.get_logger().error(f'Khong the mo camera tai {video_device}')
            sys.exit(1)

        # Set MJPG codec TRƯỚC KHI set resolution (bắt buộc cho HD/Full HD)
        fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        # Set resolution và fps
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # Set buffer size = 1 để luôn lấy frame mới nhất (giảm latency)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Lấy resolution thực tế từ camera
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))

        self.get_logger().info(f'Camera da mo: {actual_width}x{actual_height} @ {actual_fps}fps')

        # Kiểm tra resolution có đúng không
        if actual_width != width or actual_height != height:
            self.get_logger().warn(f'Resolution yeu cau: {width}x{height}, thuc te: {actual_width}x{actual_height}')

        # Publishers
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Lưu lại width và height thực tế để dùng trong camera_info
        self.width = actual_width
        self.height = actual_height

        # Tạo CameraInfo message với tham số mặc định (có thể calibrate sau)
        self.camera_info = self.create_camera_info()

        # Threaded capture để giảm giật
        self.frame = None
        self.frame_lock = threading.Lock()
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        # Timer để publish ảnh
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Camera Node da khoi dong! (Threaded capture, KHONG CAN cv_bridge)')

    def _capture_loop(self):
        """Thread liên tục đọc frame từ camera"""
        while self.running:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.frame_lock:
                    self.frame = frame

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
            raise ValueError(f"Encoding {encoding} chua duoc ho tro")

        return img_msg

    def create_camera_info(self):
        """
        Tạo CameraInfo message với tham số mặc định
        (Có thể calibrate camera sau để có tham số chính xác)
        """
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id
        camera_info.width = self.width
        camera_info.height = self.height

        # Camera matrix (3x3) - mặc định với focal length ước tính
        # Giả sử camera có góc nhìn ~60 độ
        fx = fy = self.width / (2.0 * np.tan(np.pi / 6.0))  # ~60 degree FOV
        cx = self.width / 2.0
        cy = self.height / 2.0
        camera_info.k = [fx, 0.0, cx,
                         0.0, fy, cy,
                         0.0, 0.0, 1.0]

        # Distortion model (Plumb Bob / Brown-Conrady)
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Không có distortion (có thể calibrate sau)

        # Rectification matrix (identity)
        camera_info.r = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]

        # Projection matrix (3x4)
        camera_info.p = [fx, 0.0, cx, 0.0,
                         0.0, fy, cy, 0.0,
                         0.0, 0.0, 1.0, 0.0]

        return camera_info

    def timer_callback(self):
        # Lấy frame từ thread capture
        with self.frame_lock:
            if self.frame is None:
                return
            frame = self.frame.copy()

        try:
            # Convert OpenCV image sang ROS2 Image message (KHÔNG CẦN cv_bridge)
            ros_image = self.cv2_to_imgmsg(frame, "bgr8")
            current_time = self.get_clock().now().to_msg()
            ros_image.header.stamp = current_time
            ros_image.header.frame_id = self.frame_id

            # Publish image
            self.image_publisher.publish(ros_image)

            # Publish camera_info (đồng bộ với image)
            self.camera_info.header.stamp = current_time
            self.camera_info_publisher.publish(self.camera_info)
        except Exception as e:
            self.get_logger().error(f'Loi convert anh: {str(e)}')

    def destroy_node(self):
        self.running = False
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
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
            node.get_logger().error(f'Loi trong node: {str(e)}')
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
