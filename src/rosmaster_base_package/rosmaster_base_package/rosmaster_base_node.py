#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField, JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from Rosmaster_Lib import Rosmaster


class RosmasterBaseNode(Node):
    def __init__(self):
        super().__init__('rosmaster_base_node')

        # ===== 파라미터 =====
        self.declare_parameter('wheel_radius', 0.105)      # 바퀴 반지름 (m)
        self.declare_parameter('track_width', 0.483)       # 좌/우 바퀴 간 거리 (m)
        self.declare_parameter('ticks_per_rev', 8896.0)    # 바퀴 1회전당 엔코더 tick 수
        self.declare_parameter('imu_link', 'imu_link')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        # 엔코더 tick 부호 (전진 시 tick 이 증가면 1.0, 감소면 -1.0)
        self.declare_parameter('enc_sign', -1.0)
        # 회전량 보정 계수 (실측 대비 과대/과소 회전을 맞추기 위해 기본 0.8 적용)
        self.declare_parameter('yaw_scale', 0.725)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.imu_link = self.get_parameter('imu_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.enc_sign = float(self.get_parameter('enc_sign').value)
        self.yaw_scale = float(self.get_parameter('yaw_scale').value)

        self.get_logger().info(
            f"wheel_radius={self.wheel_radius}, "
            f"track_width={self.track_width}, "
            f"ticks_per_rev={self.ticks_per_rev}, "
            f"enc_sign={self.enc_sign}, "
            f"yaw_scale={self.yaw_scale}"
        )

        # ===== Rosmaster 보드 초기화 =====
        self.car = Rosmaster()
        self.car.set_car_type(4)  # X1

        try:
            self.car.create_receive_threading()
            self.get_logger().info("create_receive_threading() started")
        except Exception as e:
            self.get_logger().error(f"create_receive_threading failed: {e}")

        try:
            # 상태 자동 리포트 (속도/IMU/엔코더 등)
            self.car.set_auto_report_state(True, False)
        except Exception as e:
            self.get_logger().warn(f"set_auto_report_state failed: {e}")

        # ===== 상태 변수 =====
        self.prev_enc = None          # 이전 엔코더 값 (m1, m2, m3, m4)
        # 휠 각도 (rad) 순서: FR, FL, BR, BL (joint 이름 순서와 맞춤)
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]

        # odom 상에서의 로봇 자세
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ===== ROS 통신 =====
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_cb, 10
        )

        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 50)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 50)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 50)
        self.vol_pub = self.create_publisher(Float32, 'voltage', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # 주기 타이머 (50 Hz)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update)

    # ========== cmd_vel 콜백 ==========
    def cmd_vel_cb(self, msg: Twist):
        # 상위에서 주는 기동 명령을 그대로 Rosmaster에 전달
        self.car.set_car_motion(
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z)
        )

    # ========== 주기 업데이트 ==========
    def update(self):
        now = self.get_clock().now().to_msg()

        # ---- IMU, MAG, 배터리 ----
        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()
        voltage = self.car.get_battery_voltage()

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        self.imu_pub.publish(imu)

        mag = MagneticField()
        mag.header.stamp = now
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = float(mx)
        mag.magnetic_field.y = float(my)
        mag.magnetic_field.z = float(mz)
        self.mag_pub.publish(mag)

        self.vol_pub.publish(Float32(data=float(voltage)))

        # ---- 엔코더 읽기 ----
        # 모터1: 좌측 앞(FL), 모터2: 좌측 뒤(BL), 모터3: 우측 앞(FR), 모터4: 우측 뒤(BR)
        m1, m2, m3, m4 = self.car.get_motor_encoder()
        m1 = float(m1)
        m2 = float(m2)
        m3 = float(m3)
        m4 = float(m4)

        # 첫 루프에서는 이전값만 저장
        if self.prev_enc is None:
            self.prev_enc = (m1, m2, m3, m4)
            return

        p1, p2, p3, p4 = self.prev_enc
        d1 = m1 - p1
        d2 = m2 - p2
        d3 = m3 - p3
        d4 = m4 - p4
        self.prev_enc = (m1, m2, m3, m4)

        if self.ticks_per_rev <= 0.0:
            self.get_logger().warn("ticks_per_rev <= 0, skip odom/joints")
            return

        # ===== 엔코더 tick → 거리 변환 =====
        circ = 2.0 * math.pi * self.wheel_radius
        scale = self.enc_sign * (circ / self.ticks_per_rev)

        # 모터1~4 매핑:
        #   모터1: 좌측 앞 휠 (FL)
        #   모터2: 좌측 뒤 휠 (BL)
        #   모터3: 우측 앞 휠 (FR)
        #   모터4: 우측 뒤 휠 (BR)
        dist_fl = d1 * scale   # front-left
        dist_bl = d2 * scale   # back-left
        dist_fr = d3 * scale   # front-right
        dist_br = d4 * scale   # back-right

        # 좌/우 바퀴 평균 거리 (skid-steer 4WD)
        dist_left = 0.5 * (dist_fl + dist_bl)
        dist_right = 0.5 * (dist_fr + dist_br)

        # ===== 차동 구동 모델로 odom 계산 =====
        ds = 0.5 * (dist_right + dist_left)
        dtheta = 0.0
        if self.track_width != 0.0:
            dtheta = self.yaw_scale * (dist_right - dist_left) / self.track_width

        self.yaw += float(dtheta)
        self.x += float(ds * math.cos(self.yaw))
        self.y += float(ds * math.sin(self.yaw))

        # yaw → quaternion (z축 회전만 사용)
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)

        # ----- Odometry 메시지 -----
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = float(sy)
        odom.pose.pose.orientation.w = float(cy)

        odom.twist.twist.linear.x = float(ds / self.dt)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(dtheta / self.dt)
        self.odom_pub.publish(odom)

        # ----- TF (odom → base_footprint) -----
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(sy)
        t.transform.rotation.w = float(cy)
        self.tf_broadcaster.sendTransform(t)

        # ===== JointState (바퀴 회전 각도) =====
        # 각 바퀴: 이동거리 / 반지름 → 회전각(rad)
        # joint 이름 순서: FR, FL, BR, BL
        self.wheel_angles[0] += float(dist_fr / self.wheel_radius)  # FR
        self.wheel_angles[1] += float(dist_fl / self.wheel_radius)  # FL
        self.wheel_angles[2] += float(dist_br / self.wheel_radius)  # BR
        self.wheel_angles[3] += float(dist_bl / self.wheel_radius)  # BL

        js = JointState()
        js.header.stamp = now
        js.name = [
            'front_right_joint',
            'front_left_joint',
            'back_right_joint',
            'back_left_joint'
        ]
        js.position = [
            float(self.wheel_angles[0]),
            float(self.wheel_angles[1]),
            float(self.wheel_angles[2]),
            float(self.wheel_angles[3])
        ]
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = RosmasterBaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

