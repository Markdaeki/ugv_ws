from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_desc = FindPackageShare('rosmaster_description').find('rosmaster_description')
    pkg_bringup = FindPackageShare('rosmaster_bringup').find('rosmaster_bringup')
    urdf_file = PathJoinSubstitution([pkg_desc, 'urdf', 'ugv.urdf.xacro'])
    ekf_config = PathJoinSubstitution([pkg_bringup, 'config', 'ekf_localization.yaml'])
    use_ekf = LaunchConfiguration('use_ekf', default='true')

    # Publish odom->base TF from base node when EKF is disabled
    publish_base_tf_default = PythonExpression(["'", use_ekf, "'.lower() != 'true'"])
    publish_base_tf = LaunchConfiguration('publish_base_tf', default=publish_base_tf_default)

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

    base_node = Node(
        package='rosmaster_base_package',
        executable='rosmaster_base_node',
        name='rosmaster_base_node',
        output='screen',
        parameters=[{
            'publish_odom_tf': publish_base_tf,
        }]
    )

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

    scan_filter_node = Node(
        package='rosmaster_bringup',
        executable='scan_front_filter_node',
        name='scan_front_filter',
        output='screen',
        parameters=[{
            'lower_angle': 1.5708,  # +90 deg
            'upper_angle': 4.7124,  # +270 deg
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
            description='Run EKF to fuse IMU and odom',
        ),
        DeclareLaunchArgument(
            'publish_base_tf',
            default_value=publish_base_tf_default,
            description='Publish odom->base TF from base_node (auto-true when EKF disabled)',
        ),
        robot_state_publisher,
        base_node,
        usb_cam_node,
        lidar_node,
        scan_filter_node,
        ekf_localization_node,
    ])
