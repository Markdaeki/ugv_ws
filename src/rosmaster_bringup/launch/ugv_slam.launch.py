from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # === 1) URDF 로딩 (rosmaster_description) ===
    pkg_desc = FindPackageShare('rosmaster_description').find('rosmaster_description')
    urdf_file = PathJoinSubstitution([pkg_desc, 'urdf', 'ugv.urdf.xacro'])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
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
            'yaw_scale': 0.8,
        }]
    )

    # === 3) USB 카메라 노드 (v4l2_camera) ===
    usb_cam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
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
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 460800,
            'frame_id': 'laser_link',
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[
            ('scan', '/scan'),
        ]
    )

    # === 5) 라이다 전면 필터 ===
    scan_filter_node = Node(
        package='rosmaster_bringup',
        executable='scan_front_filter_node',
        name='scan_front_filter',
        output='screen',
        parameters=[{
            'lower_angle': 1.5708,
            'upper_angle': 4.7124,
        }]
    )

    # === 6) SLAM (slam_toolbox) ===
    pkg_bringup = FindPackageShare('rosmaster_bringup').find('rosmaster_bringup')
    slam_params = PathJoinSubstitution([pkg_bringup, 'config', 'slam_toolbox.yaml'])
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    return LaunchDescription([
        robot_state_publisher,
        base_node,
        usb_cam_node,
        lidar_node,
        scan_filter_node,
        slam_node,
    ])
