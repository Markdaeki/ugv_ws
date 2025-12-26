# UGV ROS 2 작업공간 개요

이 저장소는 ROS 2 Humble 환경에서 UGV를 구동하기 위한 런치 파일과 설정을 포함합니다. `rosmaster_bringup` 패키지로 센서와 베이스 드라이버를 올리고, `slam_toolbox`로 지도 작성까지 수행할 수 있습니다.

## 준비 사항
- ROS 2 Humble + `colcon` 빌드 환경
- 라이다(예: RPLIDAR C1), IMU, USB 카메라가 연결된 UGV
- `udev` 규칙으로 라이다 디바이스를 `/dev/rplidar` 로 바인딩한 상태 권장

## 빌드 및 실행
1. 의존 패키지가 설치되어 있다면 워크스페이스 루트에서 빌드합니다.
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

2. 기본 구동(센서 + 베이스 + EKF)이 필요하면 다음을 실행합니다.
   ```bash
   ros2 launch rosmaster_bringup ugv_bringup.launch.py use_ekf:=true publish_base_tf:=false
   ```
   - 업계 표준은 **EKF만 odom→base TF를 퍼블리시**하는 것입니다(`publish_base_tf` 기본값 `false`).
   - EKF를 끌 때(`use_ekf:=false`)에만 `publish_base_tf:=true` 로 지정해 베이스 노드가 TF를 대신 퍼블리시합니다.
   - RViz 기본 Fixed Frame은 `odom`, 표시할 오도메트리는 `/odometry/filtered` 하나만 두고 `/odom`은 비교용으로만 켜는 구성을 권장합니다.

3. 지도 작성 모드는 SLAM 노드를 포함한 런치를 사용합니다.
   ```bash
   ros2 launch rosmaster_bringup ugv_slam.launch.py
   ```
   - `ugv_slam.launch.py` 는 베이스 노드와 라이다 전면 필터(`scan_front_filter_node`), `slam_toolbox`를 함께 올립니다.

> **SLLIDAR 에러 코드 80008004가 발생할 때**
> - 전원과 케이블 연결을 다시 확인합니다 (C1은 USB 전원 부족 시 빈번).
> - `ls -l /dev/rplidar` 로 udev 바인딩이 올바른지 확인하고, 실제 포트를 `serial_port:=/dev/ttyUSB0` 처럼 런치 인자로 지정해 테스트해봅니다.
> - `sudo chmod a+rw /dev/ttyUSB0` 후 재시도해 권한 문제를 배제합니다.

## 지도 저장
SLAM 모드에서 충분히 주행한 후 `map_saver_cli` 로 지도를 저장할 수 있습니다.
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```
생성된 `my_map.pgm`, `my_map.yaml` 파일을 `ros2 launch` 시 `slam_toolbox` 또는 `nav2_map_server`에 로드해 재사용할 수 있습니다.

## 카메라 캘리브레이션 경로 문제 해결
- `v4l2_camera` 로그에서 `camera calibration file ... not found` 가 출력되면 아래 명령으로 기본 경로를 만들어 주세요.
  ```bash
  mkdir -p ~/.ros/camera_info
  ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.03 --camera_name arducam_usb
  ```
- 촬영이 끝나면 생성된 `~/.ros/camera_info/arducam_usb_camera:_arducam_usb.yaml` 파일을 워크스페이스에 백업해 두면 재설치 후에도 재사용할 수 있습니다.

## 추가 참고 문서
- `docs/odom_alignment.md`: 오도메트리 스케일/각도 보정 체크리스트
- `docs/imu_sensor_fusion.md`: IMU 융합 및 필터 설정 가이드
- `docs/rviz_sync.md`: RViz와 센서 데이터 동기화/표시 팁
