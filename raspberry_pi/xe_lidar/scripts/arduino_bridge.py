#!/usr/bin/env python3
"""
Node ROS2 Bridge để gửi lệnh cmd_vel từ ROS2 tới Arduino qua Serial

Node này:
- Subscribe topic /cmd_vel (geometry_msgs/Twist)
- Chuyển đổi linear.x và angular.z thành lệnh Serial
- Gửi lệnh tới Arduino với format: "V:linear:angular\n"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
import sys
import time


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Mặc định cho Arduino
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('auto_detect', True)  # Tự động tìm Arduino
        
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        auto_detect = self.get_parameter('auto_detect').value
        
        # Tự động tìm Arduino nếu được bật
        if auto_detect:
            detected_port = self.detect_arduino_port()
            if detected_port:
                serial_port = detected_port
                self.get_logger().info(f'Tự động phát hiện Arduino tại: {serial_port}')
        
        # Khởi tạo Serial connection
        self.serial_conn = None
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=timeout
            )
            # Đợi Arduino khởi động
            time.sleep(2)
            
            # Đọc và xóa buffer
            if self.serial_conn.in_waiting > 0:
                self.serial_conn.read_all()
            
            self.get_logger().info(f'Đã kết nối với Arduino tại {serial_port} (baudrate: {baudrate})')
        except serial.SerialException as e:
            self.get_logger().error(f'Không thể kết nối với Arduino: {str(e)}')
            self.get_logger().error(f'Đảm bảo Arduino đã được kết nối và port đúng: {serial_port}')
            sys.exit(1)
        
        # Subscriber cho cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Đã subscribe topic /cmd_vel')
        
        # Biến lưu giá trị hiện tại
        self.last_cmd_time = time.time()
        self.last_linear = 0.0
        self.last_angular = 0.0
        
        # Timer để kiểm tra timeout và gửi lệnh dừng nếu cần
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        self.get_logger().info('Arduino Bridge Node đã khởi động!')
    
    def detect_arduino_port(self):
        """Tự động phát hiện cổng Serial của Arduino"""
        ports = serial.tools.list_ports.comports()
        
        # Tìm các cổng có thể là Arduino
        for port in ports:
            # Arduino thường có description chứa "Arduino" hoặc "USB"
            desc = port.description.lower()
            if 'arduino' in desc or 'ch340' in desc or 'ch341' in desc or 'cp210' in desc:
                return port.device
        
        # Nếu không tìm thấy, thử các cổng phổ biến
        common_ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
        for port_name in common_ports:
            try:
                test_serial = serial.Serial(port_name, timeout=0.1)
                test_serial.close()
                return port_name
            except:
                continue
        
        return None
    
    def cmd_vel_callback(self, msg):
        """Callback xử lý lệnh cmd_vel từ ROS2"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Lưu thời gian và giá trị
        self.last_cmd_time = time.time()
        self.last_linear = linear
        self.last_angular = angular
        
        # Gửi lệnh tới Arduino
        self.send_command(linear, angular)
    
    def send_command(self, linear, angular):
        """Gửi lệnh tới Arduino qua Serial"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        
        try:
            # Format: "V:linear:angular\n"
            # Làm tròn để giảm dữ liệu
            linear_str = f"{linear:.3f}"
            angular_str = f"{angular:.3f}"
            command = f"V:{linear_str}:{angular_str}\n"
            
            # Gửi lệnh
            self.serial_conn.write(command.encode('utf-8'))
            self.serial_conn.flush()  # Đảm bảo dữ liệu được gửi ngay
            
            # Debug (có thể tắt)
            # self.get_logger().debug(f'Sent to Arduino: {command.strip()}')
            
        except serial.SerialTimeoutException:
            self.get_logger().warn('Timeout khi gửi lệnh tới Arduino')
        except serial.SerialException as e:
            self.get_logger().error(f'Lỗi Serial: {str(e)}')
            # Thử kết nối lại
            self.reconnect_serial()
    
    def reconnect_serial(self):
        """Thử kết nối lại Serial nếu bị ngắt"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except:
                pass
        
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                timeout=timeout,
                write_timeout=timeout
            )
            time.sleep(1)
            self.get_logger().info('Đã kết nối lại với Arduino')
        except Exception as e:
            self.get_logger().error(f'Không thể kết nối lại: {str(e)}')
    
    def timer_callback(self):
        """Timer callback để xử lý timeout"""
        current_time = time.time()
        
        # Nếu không nhận được lệnh trong 0.5 giây, gửi lệnh dừng
        if current_time - self.last_cmd_time > 0.5:
            if abs(self.last_linear) > 0.01 or abs(self.last_angular) > 0.01:
                # Gửi lệnh dừng
                self.send_command(0.0, 0.0)
                self.last_linear = 0.0
                self.last_angular = 0.0
                self.get_logger().debug('Timeout - Gửi lệnh dừng tới Arduino')
    
    def destroy_node(self):
        """Cleanup khi node bị hủy"""
        # Gửi lệnh dừng trước khi đóng
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.send_command(0.0, 0.0)
                time.sleep(0.1)
                self.serial_conn.close()
                self.get_logger().info('Đã đóng kết nối Serial')
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = ArduinoBridge()
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

