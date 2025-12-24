# RViz와 실제 로봇 움직임 동기화 가이드

실제 움직임과 RViz 시각화를 최대한 일치시키기 위해 점검해야 할 항목을 정리했습니다. 모두 ROS2 Humble + 이 워크스페이스(`ugv_slam.launch.py`) 기준입니다.

## 1. 시간 소스(use_sim_time) 통일
- 모든 노드에서 `use_sim_time`을 동일하게 설정합니다. 실제 하드웨어 기준이면 **반드시 `false`** 여야 합니다.
- 확인 명령:
  ```bash
  ros2 param get /rosmaster_base_node use_sim_time
  ros2 param get /sllidar_node use_sim_time
  ros2 param get /slam_toolbox use_sim_time
  ```

## 2. TF 발행 중복 제거
- `robot_state_publisher`는 **한 개만 실행**되도록 합니다.
- 중복 확인:
  ```bash
  ros2 node list | grep robot_state_publisher
  ```
- 만약 여러 개가 뜨면 불필요한 런치를 종료하거나, bringup/slam 런치를 하나만 유지합니다.

## 3. TF 체인과 주기 확인
- 필수 체인: `map -> odom -> base_footprint -> base_link -> laser_link`
- 주기 확인:
  ```bash
  ros2 topic hz /tf
  ros2 topic hz /odom
  ros2 topic hz /scan_front
  ```
- `/tf`가 너무 느리거나 스파이크가 있으면 RViz에서 모델이 튈 수 있습니다. 베이스 노드나 라이다 노드의 퍼블리시 주기를 점검하세요.

## 4. SLAM/Localization 파라미터에서 TF 버퍼 여유 확보
- `src/rosmaster_bringup/config/slam_toolbox.yaml`에 `tf_buffer_duration: 10.0`을 설정했습니다. TF가 약간 늦게 도착해도 `/map` 계산이 끊기지 않도록 완충 역할을 합니다.

## 5. LaserScan 큐 사이즈와 Fixed Frame 점검
- RViz에서 `LaserScan` 디스플레이의 **Queue Size**를 넉넉히(예: 10~20) 설정합니다.
- Fixed Frame은 **`map`** 으로 설정합니다. 로봇 모델이 튀거나 안 보이면 `TF` 디스플레이에서 체인을 확인하세요.

## 6. 오도메트리 품질 확인
- `/odom` 값이 순간적으로 튀면 RViz 모델도 함께 튑니다. 하드웨어 드라이버나 휠 이득(`yaw_scale`, `speed_limit`)을 점검하고, 필요하면 필터를 적용하세요.

## 7. 흔한 증상별 빠른 조치
- **TF/RobotModel가 왔다 갔다 함**: `robot_state_publisher` 중복 실행 여부와 `/tf` 스파이크 확인.
- **LaserScan이 자주 드롭됨**: `Queue Size` 확대, `/tf` 주기 안정화, 라이다와 SLAM의 토픽 이름 일치(`/scan_front`).
- **RViz에서 지도/로봇이 멈춰 보임**: `/map`, `/tf`, `/odom` 세 토픽이 모두 정상인지 `ros2 topic list`와 `ros2 topic echo`로 확인.

위 항목을 순서대로 점검하면 실측 움직임과 RViz 표현이 대부분 일치합니다. 문제가 지속되면 `/tf` 로그와 노드/파라미터 상태를 함께 확인해 주세요.
