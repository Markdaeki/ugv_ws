#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFrontFilterNode(Node):
    def __init__(self):
        super().__init__('scan_front_filter')

        # 파라미터: 앞쪽 범위 (rad)
        #
        # 라이다 조인트를 180도 회전시켜 base_link와 정렬한 상태이므로
        # 로봇의 "정면"을 선택하려면 스캔 좌표계 기준으로 π/2 ~ 3π/2를
        # 필터링해야 한다.
        self.declare_parameter('lower_angle',  math.pi / 2.0)      # +90 deg
        self.declare_parameter('upper_angle',  3.0 * math.pi / 2.0)  # +270 deg

        self.lower_angle = float(self.get_parameter('lower_angle').value)
        self.upper_angle = float(self.get_parameter('upper_angle').value)

        # 입력: /scan_raw (RPLIDAR 원본)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10
        )

        # 출력: /scan
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.get_logger().info(
            f"ScanFrontFilterNode started: "
            f"lower_angle={self.lower_angle:.3f}, upper_angle={self.upper_angle:.3f}"
        )

    def scan_callback(self, msg: LaserScan):
        # LaserScan 각도 범위
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment

        # 인덱스 범위 계산
        # idx = round((angle - angle_min) / angle_inc)
        start_idx = int(round((self.lower_angle - angle_min) / angle_inc))
        end_idx   = int(round((self.upper_angle - angle_min) / angle_inc))

        # 인덱스 범위 클램핑
        start_idx = max(0, min(start_idx, len(msg.ranges) - 1))
        end_idx   = max(0, min(end_idx, len(msg.ranges) - 1))

        if start_idx >= end_idx:
            # 범위가 말이 안 되면 그냥 통과
            self.scan_pub.publish(msg)
            return

        # 새 LaserScan 메시지 생성
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.header.stamp = self.get_clock().now().to_msg()

        new_msg.angle_min = angle_min + start_idx * angle_inc
        new_msg.angle_max = angle_min + end_idx   * angle_inc
        new_msg.angle_increment = angle_inc

        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max

        new_msg.ranges = msg.ranges[start_idx:end_idx+1]
        if msg.intensities:
            new_msg.intensities = msg.intensities[start_idx:end_idx+1]
        else:
            new_msg.intensities = []

        self.scan_pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrontFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
