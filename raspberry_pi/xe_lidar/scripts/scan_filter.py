#!/usr/bin/env python3
"""
Node filter LaserScan - chi giu lai goc phia truoc
Subscribe: /scan
Publish: /scan_filtered (chi giu lai goc truoc)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')

        # Parameters
        self.declare_parameter('front_angle', 60.0)  # Goc phia truoc (do)
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')

        self.front_angle = self.get_parameter('front_angle').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.front_angle_rad = math.radians(self.front_angle / 2)  # Nua goc moi ben

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10
        )

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, output_topic, 10)

        self.get_logger().info(f'Scan Filter: {input_topic} -> {output_topic}')
        self.get_logger().info(f'Chi giu lai {self.front_angle} do phia truoc')

    def scan_callback(self, msg):
        """Filter scan data chi giu lai goc phia truoc"""
        # Tao message moi
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header

        # Tinh chi so bat dau va ket thuc cho goc phia truoc
        # Goc 0 la phia truoc, goc am la ben phai, goc duong la ben trai
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_readings = len(msg.ranges)

        # Tim chi so cho -front_angle/2 va +front_angle/2
        start_angle = -self.front_angle_rad
        end_angle = self.front_angle_rad

        # Tinh chi so
        start_idx = int((start_angle - angle_min) / angle_increment)
        end_idx = int((end_angle - angle_min) / angle_increment)

        # Dam bao chi so hop le
        start_idx = max(0, min(start_idx, num_readings - 1))
        end_idx = max(0, min(end_idx, num_readings - 1))

        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx

        # Tao scan moi voi chi goc phia truoc
        filtered_msg.angle_min = start_angle
        filtered_msg.angle_max = end_angle
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max

        # Copy ranges va intensities cho goc phia truoc
        filtered_msg.ranges = list(msg.ranges[start_idx:end_idx + 1])
        if msg.intensities:
            filtered_msg.intensities = list(msg.intensities[start_idx:end_idx + 1])

        # Publish
        self.scan_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
