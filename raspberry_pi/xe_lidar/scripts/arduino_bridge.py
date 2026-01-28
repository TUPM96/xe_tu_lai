#!/usr/bin/env python3
"""
Node ROS2 Bridge để gửi lệnh cmd_vel từ ROS2 tới Arduino qua Serial

Node này:
- Subscribe topic /cmd_vel (geometry_msgs/Twist)
- Chuyển đổi linear.x và angular.z thành lệnh Serial
- Gửi lệnh tới Arduino với format: "V:linear:angular\n"
"""

import rclpy
from rclpy import logging
from rclpy.node import Node
from geometry_msgs import msg as geom_msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import serial
import serial.tools.list_ports
import sys
import time
import math


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Mặc định cho Arduino
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('auto_detect', True)  # Tự động tìm Arduino
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_odom', True)  # Bật/tắt publish odometry
        self.declare_parameter('max_linear_speed', 0.3)  # Tốc độ tối đa (m/s) gửi tới Arduino
        self.declare_parameter('motor_min_pwm', 100)  # PWM tối thiểu (0-255) gửi tới Arduino
        
        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        auto_detect = self.get_parameter('auto_detect').value
        
        # Tự động tìm Arduino chỉ nếu được bật VÀ port được chỉ định không tồn tại
        if auto_detect:
            import os
            if not os.path.exists(serial_port):
                # Port được chỉ định không tồn tại, thử auto-detect
                detected_port = self.detect_arduino_port()
                if detected_port:
                    serial_port = detected_port
                    self.get_logger().info(f'Port được chỉ định không tồn tại. Tự động phát hiện Arduino tại: {serial_port}')
            else:
                # Port được chỉ định tồn tại, dùng port đó
                self.get_logger().info(f'Sử dụng port được chỉ định: {serial_port}')
        
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

            # Gửi config tốc độ tới Arduino
            self.send_motor_config()
        except serial.SerialException as e:
            self.get_logger().error(f'Không thể kết nối với Arduino: {str(e)}')
            self.get_logger().error(f'Đảm bảo Arduino đã được kết nối và port đúng: {serial_port}')
            sys.exit(1)
        
        # Subscriber cho cmd_vel (điều khiển vận tốc)
        self.cmd_vel_sub = self.create_subscription(
            geom_msg.Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Đã subscribe topic /cmd_vel')

        # Subscriber cho lệnh góc servo trực tiếp (S:angle)
        self.servo_cmd_sub = self.create_subscription(
            Float32,
            '/servo_angle_cmd',
            self.servo_cmd_callback,
            10
        )
        self.get_logger().info('Đã subscribe topic /servo_angle_cmd')
        
        # Parameters cho odometry
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_odom = self.get_parameter('publish_odom').value
        
        # Odometry publisher và transform broadcaster
        if self.publish_odom:
            self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
            self.tf_broadcaster = TransformBroadcaster(self)
            self.get_logger().info('Đã tạo Odometry publisher và TF broadcaster')

        # Joint State publisher
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info('Đã tạo Joint State publisher')

        # Robot parameters (phải khớp với URDF)
        self.wheelbase = 0.4  # Khoảng cách trục trước-sau (m)
        self.wheel_radius = 0.034  # Bán kính bánh xe (m)
        self.max_steer_angle = 0.5236  # Góc lái tối đa (rad) ~30 độ

        # Joint states
        self.steering_angle = 0.0  # Góc lái hiện tại
        self.wheel_position = 0.0  # Vị trí bánh xe (tích lũy)
        
        # Biến cho odometry (dead reckoning từ cmd_vel)
        self.x = 0.0  # Vị trí x (m)
        self.y = 0.0  # Vị trí y (m)
        self.theta = 0.0  # Góc quay (rad)
        self.last_odom_time = self.get_clock().now()
        
        # Biến lưu giá trị hiện tại
        self.last_cmd_time = time.time()
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.timeout_sent = False  # Flag để tránh gửi lệnh dừng liên tục
        
        # Timer để kiểm tra timeout, gửi lệnh dừng, và publish odometry
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        self.get_logger().info('Arduino Bridge Node đã khởi động!')
    
    def detect_arduino_port(self):
        """Tự động phát hiện cổng Serial của Arduino"""
        import os
        
        # Ưu tiên ttyACM (Arduino thường dùng ttyACM, LiDAR thường dùng ttyUSB)
        # Kiểm tra ttyACM trước
        for port_name in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']:
            if os.path.exists(port_name):
                try:
                    test_serial = serial.Serial(port_name, timeout=0.1)
                    test_serial.close()
                    return port_name
                except:
                    continue
        
        # Tìm các cổng có thể là Arduino qua description
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Arduino thường có description chứa "Arduino", "CH340", "CH341", "CP210"
            desc = port.description.lower()
            # Ưu tiên ttyACM hơn ttyUSB
            if port.device.startswith('/dev/ttyACM') and ('arduino' in desc or 'ch340' in desc or 'ch341' in desc or 'cp210' in desc):
                return port.device
        
        # Cuối cùng mới thử ttyUSB (thường là LiDAR hoặc thiết bị khác)
        for port_name in ['/dev/ttyUSB0', '/dev/ttyUSB1']:
            if os.path.exists(port_name):
                try:
                    test_serial = serial.Serial(port_name, timeout=0.1)
                    test_serial.close()
                    return port_name
                except:
                    continue
        
        return None
    
    def send_motor_config(self):
        """Gửi config tốc độ tới Arduino (lệnh M:)"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return

        max_speed = self.get_parameter('max_linear_speed').value
        min_pwm = self.get_parameter('motor_min_pwm').value

        try:
            # Format: "M:max_speed:min_pwm\n"
            command = f"M:{max_speed:.2f}:{min_pwm}\n"
            self.serial_conn.write(command.encode('utf-8'))
            self.serial_conn.flush()
            self.get_logger().info(f'Đã gửi config Arduino: max_speed={max_speed} m/s, min_pwm={min_pwm}')

            # Đọc response từ Arduino
            time.sleep(0.1)
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                self.get_logger().info(f'Arduino response: {response.strip()}')
        except Exception as e:
            self.get_logger().warn(f'Không thể gửi config tới Arduino: {str(e)}')

    def cmd_vel_callback(self, msg: geom_msg.Twist):
        """Callback xử lý lệnh cmd_vel từ ROS2"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Lưu thời gian và giá trị
        self.last_cmd_time = time.time()
        self.last_linear = linear
        self.last_angular = angular
        self.timeout_sent = False  # Reset timeout flag khi nhận lệnh mới
        
        # Gửi lệnh tới Arduino
        self.send_command(linear, angular)
    
    def send_command(self, linear: float, angular: float):
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
        """Timer callback để xử lý timeout và publish odometry"""
        current_time = time.time()
        
        # Nếu không nhận được lệnh trong 0.5 giây, gửi lệnh dừng
        # Chỉ gửi một lần để tránh spam Serial
        if current_time - self.last_cmd_time > 0.5:
            if not self.timeout_sent and (abs(self.last_linear) > 0.01 or abs(self.last_angular) > 0.01):
                # Gửi lệnh dừng (Arduino cũng có timeout riêng)
                self.send_command(0.0, 0.0)
                self.last_linear = 0.0
                self.last_angular = 0.0
                self.timeout_sent = True
                self.get_logger().debug('Timeout - Gửi lệnh dừng tới Arduino')
        else:
            # Reset flag khi có lệnh mới
            self.timeout_sent = False
        
        # Publish odometry từ cmd_vel (dead reckoning)
        if self.publish_odom:
            self.update_and_publish_odometry()

        # Publish joint states
        self.publish_joint_states()
    
    def update_and_publish_odometry(self):
        """Tính toán và publish odometry từ cmd_vel (dead reckoning)"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9  # Convert to seconds
        
        if dt < 0.001:  # Tránh chia cho số quá nhỏ
            return
        
        # Lấy vận tốc hiện tại
        v = self.last_linear  # linear velocity (m/s)
        omega = self.last_angular  # angular velocity (rad/s)
        
        # Tính toán vị trí mới (dead reckoning)
        if abs(omega) < 0.001:  # Đi thẳng
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        else:  # Đi cong
            # Bán kính quay
            radius = v / omega if abs(omega) > 0.001 else 0.0
            # Góc quay trong dt
            dtheta = omega * dt
            # Vị trí mới
            self.x += radius * (math.sin(self.theta + dtheta) - math.sin(self.theta))
            self.y += radius * (-math.cos(self.theta + dtheta) + math.cos(self.theta))
            self.theta += dtheta
        
        # Normalize theta về [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Tạo Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        
        # Vị trí
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Quaternion từ yaw
        q = self.euler_to_quaternion(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation = q
        
        # Vận tốc
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = omega
        
        # Covariance (đặt giá trị mặc định - có thể điều chỉnh)
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[35] = 0.1  # yaw
        odom_msg.twist.covariance[0] = 0.1  # vx
        odom_msg.twist.covariance[35] = 0.1  # vyaw
        
        # Publish odometry
        self.odom_publisher.publish(odom_msg)
        
        # Publish transform odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(t)

    def send_servo_angle(self, angle_deg: float):
        """Gửi lệnh góc servo trực tiếp tới Arduino (S:angle)"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        try:
            angle_int = int(round(angle_deg))
            if angle_int < 0:
                angle_int = 0
            if angle_int > 180:
                angle_int = 180
            command = f"S:{angle_int}\n"
            self.serial_conn.write(command.encode('utf-8'))
            self.serial_conn.flush()
            # self.get_logger().debug(f'Sent servo cmd: {command.strip()}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Lỗi Serial khi gửi S:angle: {e}')

    def servo_cmd_callback(self, msg: Float32):
        """Nhận lệnh góc servo từ topic /servo_angle_cmd và gửi S:angle tới Arduino"""
        self.send_servo_angle(msg.data)

    def publish_joint_states(self):
        """Publish joint states dựa trên cmd_vel"""
        current_time = self.get_clock().now()

        # Tính góc lái từ angular velocity (Ackermann steering)
        v = self.last_linear
        omega = self.last_angular

        if abs(v) > 0.01 and abs(omega) > 0.01:
            # steering_angle = atan(wheelbase * omega / v)
            self.steering_angle = math.atan(self.wheelbase * omega / v)
            # Giới hạn góc lái
            self.steering_angle = max(-self.max_steer_angle,
                                      min(self.max_steer_angle, self.steering_angle))
        elif abs(v) < 0.01:
            # Đứng yên, giữ góc lái hiện tại
            pass

        # Tính vị trí bánh xe (tích lũy góc quay)
        dt = 0.1  # 10 Hz
        wheel_angular_velocity = v / self.wheel_radius  # rad/s
        self.wheel_position += wheel_angular_velocity * dt

        # Normalize wheel position về [-pi, pi] để tránh overflow
        while self.wheel_position > math.pi:
            self.wheel_position -= 2 * math.pi
        while self.wheel_position < -math.pi:
            self.wheel_position += 2 * math.pi

        # Tạo JointState message
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.header.frame_id = ''

        # Tên các joints (phải khớp với URDF)
        joint_state.name = [
            'steering_wheel_joint',
            'front_left_steering_joint',
            'front_right_steering_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]

        # Vị trí các joints
        joint_state.position = [
            self.steering_angle,       # steering_wheel_joint
            self.steering_angle,       # front_left_steering_joint
            self.steering_angle,       # front_right_steering_joint
            self.wheel_position,       # front_left_wheel_joint
            self.wheel_position,       # front_right_wheel_joint
            self.wheel_position,       # rear_left_wheel_joint
            self.wheel_position        # rear_right_wheel_joint
        ]

        # Vận tốc các joints
        joint_state.velocity = [
            0.0,                       # steering_wheel_joint
            0.0,                       # front_left_steering_joint
            0.0,                       # front_right_steering_joint
            wheel_angular_velocity,    # front_left_wheel_joint
            wheel_angular_velocity,    # front_right_wheel_joint
            wheel_angular_velocity,    # rear_left_wheel_joint
            wheel_angular_velocity     # rear_right_wheel_joint
        ]

        # Publish
        self.joint_state_pub.publish(joint_state)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Chuyển đổi Euler angles (roll, pitch, yaw) sang Quaternion"""
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q
    
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

