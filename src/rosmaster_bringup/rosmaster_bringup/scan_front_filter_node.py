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

        # 로봇 기준 전면 반각(도). 기본 90도 => 전면 180도
        self.declare_parameter('front_half_angle_deg', 90.0)

        # URDF에서 laser_joint yaw=PI를 적용했으므로 기본 오프셋 PI
        self.declare_parameter('angle_offset', math.pi)

        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info("ScanFrontFilterNode started (masking mode)")

    def cb(self, msg: LaserScan):
        half = math.radians(float(self.get_parameter('front_half_angle_deg').value))
        offset = float(self.get_parameter('angle_offset').value)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        INF = float('inf')
        ranges = list(msg.ranges)

        n = len(ranges)
        for i in range(n):
            a = msg.angle_min + i * msg.angle_increment  # laser_link 기준
            a_robot = wrap_to_pi(a + offset)             # 로봇 기준으로 보정(PI)

            # 로봇 전면(-half~+half)만 남기고, 나머지는 무효화
            if abs(a_robot) > half:
                ranges[i] = INF

        out.ranges = ranges

        # intensities는 길이 유지(선택)
        if msg.intensities:
            intens = list(msg.intensities)
            for i in range(len(intens)):
                a = msg.angle_min + i * msg.angle_increment
                a_robot = wrap_to_pi(a + offset)
                if abs(a_robot) > half:
                    intens[i] = 0.0
            out.intensities = intens
        else:
            out.intensities = []

        self.pub.publish(out)

def main():
    rclpy.init()
    node = ScanFrontFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
