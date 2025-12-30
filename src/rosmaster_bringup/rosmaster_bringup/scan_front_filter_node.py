#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class ScanFrontFilterNode(Node):
    def __init__(self):
        super().__init__('scan_front_filter')

        # 로봇 전면 범위: -90~+90 (라디안)
        self.declare_parameter('lower_angle', -math.pi/2)
        self.declare_parameter('upper_angle',  math.pi/2)

        # URDF에서 laser_link를 yaw=PI로 뒤집어 둔 상태 보정용 오프셋
        # (기본값 PI 권장)
        self.declare_parameter('angle_offset', math.pi)

        self.lower_angle = float(self.get_parameter('lower_angle').value)
        self.upper_angle = float(self.get_parameter('upper_angle').value)
        self.angle_offset = float(self.get_parameter('angle_offset').value)

        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info(
            f"ScanFrontFilterNode started: lower={self.lower_angle:.3f}, upper={self.upper_angle:.3f}, "
            f"offset={self.angle_offset:.3f}"
        )

    def scan_callback(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header

        # 각도/메타 정보는 원본 그대로 유지 (연속성 보장)
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        # ranges 길이 유지하면서 전면만 남기기(나머지는 무효화)
        n = len(msg.ranges)
        out_ranges = list(msg.ranges)

        # 전면이 아니면 "관측 없음" 처리
        # 보통 inf가 가장 안전합니다( RViz/AMCL/Costmap에서 무시되는 편 )
        INF = float('inf')

        for i in range(n):
            a = msg.angle_min + i * msg.angle_increment
            # URDF 뒤집힘 보정(로봇 기준 각도로 변환)
            a_robot = wrap_to_pi(a + self.angle_offset)

            if not (self.lower_angle <= a_robot <= self.upper_angle):
                out_ranges[i] = INF

        out.ranges = out_ranges

        # intensities도 길이 맞춰 유지(있으면)
        if msg.intensities:
            out.intensities = list(msg.intensities)
            # 전면 아닌 곳 intensity 0으로(선택 사항)
            for i in range(n):
                a = msg.angle_min + i * msg.angle_increment
                a_robot = wrap_to_pi(a + self.angle_offset)
                if not (self.lower_angle <= a_robot <= self.upper_angle):
                    out.intensities[i] = 0.0
        else:
            out.intensities = []

        self.scan_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFrontFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
