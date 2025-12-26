from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # === 1) URDF 로딩 (rosmaster_description) ===
    pkg_desc = FindPackageShare('rosmaster_description').find('rosmaster_description')
    pkg_bringup = FindPackageShare('rosmaster_bringup').find('rosmaster_bringup')
    urdf_file = PathJoinSubstitution([pkg_desc, 'urdf', 'ugv.urdf.xacro'])
    ekf_config = PathJoinSubstitution([pkg_bringup, 'config', 'ekf_localization.yaml'])
    use_ekf = LaunchConfiguration('use_ekf', default='true')
    # EKF를 끌 때 odom->base TF가 사라지지 않도록 기본값을 use_ekf의 반대 값으로 설정
<<<<<<< ours
    publish_base_tf_default = PythonExpression(['"true" if not ', use_ekf, ' else "false"'])
=======
    publish_base_tf_default = PythonExpression([
        "not ('",
        use_ekf,
        "').lower() == 'true'",
    ])
>>>>>>> theirs
    publish_base_tf = LaunchConfiguration('publish_base_tf', default=publish_base_tf_default)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            # ★★ 여기 중요: 'xacro' 와 urdf_file 을 리스트의 "두 원소"로 분리 ★★
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            ),
        }]
    )

    # === 2) 베이스 드라이버 노드 (rosmaster_base_package) ===
    base_node = Node(
        package='rosmaster_base_package',
        executable='rosmaster_base_node',
        name='rosmaster_base_node',
        output='screen',
        parameters=[{
            # 필요한 경우에만 파라미터 추가, 일단 비워둬도 됨
            # 'wheel_radius': 0.095,
            # 'track_width': 0.60375,
            # 'ticks_per_rev': 8896.0,
            # 'enc_sign': -1.0,
            #'yaw_scale': 0.8,  # 실측 360도 회전에 RViz 450도 -> 0.8로 보정
            'publish_odom_tf': ParameterValue(publish_base_tf, value_type=bool),
        }]
    )

    # === 3) USB 카메라 노드 (v4l2_camera) ===
    usb_cam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',   # 필요시 /dev/video1 등으로 변경
            'frame_id': 'camera_link',
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
        ]
    )

    # === 4) RPLIDAR C1 노드 (sllidar_ros2) ===
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_c1',
        output='screen',
        parameters=[{
            'serial_port': '/dev/rplidar',   # udev로 바인딩해둔 디바이스
            'serial_baudrate': 460800,       # 
            'frame_id': 'laser_link',
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[
            ('scan', '/scan'),
        ]
    )
    
    #lidat filter
    scan_filter_node = Node(
        package='rosmaster_bringup',
        executable='scan_front_filter_node',
        name='scan_front_filter',
        output='screen',
        parameters=[{
            # 라이다를 URDF에서 180도 회전했으므로 스캔 좌표계 기준으로
            # +90도 ~ +270도(π/2 ~ 3π/2)를 전면으로 사용한다.
            'lower_angle':  1.5708,  # +90도
            'upper_angle':  4.7124,  # +270도
        }]
    )

    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('imu/data', '/imu/data_raw'),
            ('odometry/filtered', '/odometry/filtered'),
        ],
        condition=IfCondition(use_ekf),
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='IMU와 휠 오dom을 EKF로 융합할지 여부'
        ),
        DeclareLaunchArgument(
            'publish_base_tf',
            default_value=publish_base_tf_default,
            description='EKF를 끌 때 베이스 노드가 odom->base TF를 퍼블리시하도록 자동 전환'
        ),
        robot_state_publisher,
        base_node,
        usb_cam_node,
        lidar_node,
        scan_filter_node,
        ekf_localization_node,
    ])

