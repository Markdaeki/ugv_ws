#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

def wrap_to_pi(a: float) -> float:
    # (-pi, pi]로 정규화
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class ScanFrontFilterNode(Node):
    def __init__(self):
        super().__init__('scan_front_filter')

        # 로봇 기준 전면 범위: -90~+90 deg
        self.declare_parameter('lower_angle', -math.pi/2)  # -90
        self.declare_parameter('upper_angle',  math.pi/2)  # +90
        self.lower_angle = float(self.get_parameter('lower_angle').value)
        self.upper_angle = float(self.get_parameter('upper_angle').value)

        # 입력/출력
        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info(
            f"ScanFrontFilterNode started: lower_angle={self.lower_angle:.3f}, upper_angle={self.upper_angle:.3f}"
        )

    def scan_callback(self, msg: LaserScan):
        a_min = msg.angle_min
        inc = msg.angle_increment
        n = len(msg.ranges)

        selected_ranges = []
        selected_intensities = [] if msg.intensities else None
        selected_angles = []

        for i in range(n):
            a = a_min + i * inc

            # URDF에서 laser_link를 yaw=PI로 뒤집어둔 상태 보정
            # (로봇 전면 기준 각도로 바꾼 뒤 필터링)
            aw = wrap_to_pi(a + math.pi)

            if self.lower_angle <= aw <= self.upper_angle:
                selected_ranges.append(msg.ranges[i])
                if selected_intensities is not None:
                    selected_intensities.append(msg.intensities[i])
                selected_angles.append(aw)

        if len(selected_ranges) < 2:
            return

        out = LaserScan()
        out.header = msg.header  # stamp/frame 유지

        # 선택된 각도 범위로 갱신
        out.angle_min = min(selected_angles)
        out.angle_max = max(selected_angles)
        out.angle_increment = inc

        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        out.ranges = selected_ranges
        out.intensities = selected_intensities if selected_intensities is not None else []

        self.scan_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFrontFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
