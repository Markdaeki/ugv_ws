# IMU 센서퓨전(EKF) 구성 가이드

`rosmaster_base_node`가 퍼블리시하는 `imu/data_raw`(가속도/자이로)와 `odom`(휠 오dom)을 `robot_localization` EKF로 융합해 더 안정적인 자세/속도를 얻는 방법을 정리했습니다.

## 1. 준비물
- ROS 2용 `robot_localization` 패키지가 설치돼 있어야 합니다. (예: `sudo apt install ros-$ROS_DISTRO-robot-localization`)
- IMU 링크 이름(`imu_link`), 오dom 프레임(`odom`), 바디 프레임(`base_footprint`)이 URDF/드라이버 파라미터와 일치해야 합니다. 기본 값이 이미 맞춰져 있으니 특별한 변경이 없다면 그대로 사용하면 됩니다.

## 2. EKF 설정 파일
`rosmaster_bringup/config/ekf_localization.yaml`에 EKF 노드 설정을 추가했습니다.
- IMU는 `imu/data_raw`에서 **각속도 + 선가속도**를 사용하도록 `imu0_config`를 구성했습니다.
- 휠 오dom(`odom`)은 **평면(x, y) 위치·yaw**에 더해 **직선 속도(vx)와 yaw rate**까지 사용하도록 `odom0_config`를 켰습니다. 이미 휠 오dom이 적분된 포즈를 내보내므로 `odom0_differential`은 끕니다.
- 2D 지면 주행을 가정해 `two_d_mode: true`를 켰고, 기존 TF와 충돌하지 않도록 `publish_tf: false`로 설정했습니다. 융합된 포즈는 `odometry/filtered` 토픽에서 읽을 수 있습니다.

## 3. 실행 방법
`ugv_bringup.launch.py`에 EKF 노드를 추가해 기본적으로 활성화해 두었습니다. 필요에 따라 런치 인자로 끌 수 있습니다.

```bash
# EKF 활성(기본) 상태로 실행
ros2 launch rosmaster_bringup ugv_bringup.launch.py

# EKF를 끄고 싶을 때
ros2 launch rosmaster_bringup ugv_bringup.launch.py use_ekf:=false
```

### 토픽 흐름 확인
- 입력: `/imu/data_raw`, `/odom`
- 출력: `/odometry/filtered` (EKF 융합 결과)

Rviz에서 `odometry/filtered`를 시각화하고 `/imu/data_raw`와 `/odom`이 정상적으로 들어오는지 확인하면 됩니다. 필요하면 EKF 파라미터를 조정해도 됩니다(예: `imu0_remove_gravitational_acceleration`, 잡음 공분산 등).
dd