# 지도와 실제 이동 거리 일치시키기 (Odometry/지도 스케일 보정)

실제 로봇 이동 거리와 RViz(지도/odom) 이동 거리가 맞지 않을 때 수행하는 체크리스트입니다. 
SLAM으로 지도 저장을 끝낸 뒤 **Localization 모드**에서 점검하는 것을 권장합니다.

---

## 0. 사전 조건
- `slam_toolbox`는 **localization 모드**만 실행하고, SLAM(지도작성) 노드는 끈 상태.
- 센서/베이스 노드는 정상 실행 중 (`/odom`, `/scan_front`, TF 체인 유지).
- TF 중복이 없는지 확인 (`robot_state_publisher` 1개만 실행).

---

## 1. 직선 거리 보정 (선속도 스케일)
1. 바닥에 **정확한 기준 거리**(예: 2 m)를 표시하고 로봇을 직선 주행.
2. 주행 전/후의 `/odom` 또는 RViz 상 **기준 좌표 차이**(x, y 변위)를 기록.
3. 실제 이동 거리 대비 RViz 이동 거리를 비교:
   - RViz 거리가 짧으면 → 속도 스케일이 낮음 → **wheel_radius를 키우거나 ticks_per_rev를 줄임**.
   - RViz 거리가 길면 → 속도 스케일이 높음 → **wheel_radius를 줄이거나 ticks_per_rev를 늘림**.
4. 파라미터는 `rosmaster_base_node`가 사용합니다:
   - `wheel_radius` (기본 0.095 m)
   - `ticks_per_rev` (기본 8896.0)
5. 임시 적용 예시(런치 실행 중):
   ```bash
   ros2 param set /rosmaster_base_node wheel_radius 0.098
   ros2 param set /rosmaster_base_node ticks_per_rev 9000
   ```
   값을 바꾼 뒤 동일 거리 재주행 → 스케일이 맞을 때까지 반복.

### 조정 팁
- **wheel_radius** ↑ ⇒ 선속도/주행 거리 ↑
- **ticks_per_rev** ↑ ⇒ 선속도/주행 거리 ↓
- 두 값을 동시에 크게 바꾸지 말고, 작은 단위로 한 번에 하나씩 조정.

---

## 2. 회전(각도) 보정
1. **제자리 360° 회전** 후 `/odom`의 회전량(혹은 RViz 기준 heading 변화)을 확인.
2. 실제 회전보다 RViz가 덜/많이 돈다면 직선 보정과 동일한 두 파라미터로 조정합니다.
   - 바퀴 반지름/엔코더 스케일이 맞지 않으면 각도도 함께 틀어집니다.
3. URDF 상 바퀴 위치가 잘못된 경우에도 각도가 틀어질 수 있으므로, 메카넘/디퍼런셜 구성치가 맞는지 확인합니다.

---

## 3. 결과 고정 (재시작 후에도 유지)
임시 파라미터로 스케일이 맞춰졌다면, 런치/파라미터 파일에 영구 반영합니다. 예:

```yaml
# 예시: src/rosmaster_bringup/config/base_params.yaml
rosmaster_base_node:
  ros__parameters:
    wheel_radius: 0.098
    ticks_per_rev: 9000
```

런치에서 `params_file`로 위 파일을 불러오거나, `ugv_bringup`/`ugv_slam` 런치에 `parameters=[...]`로 지정합니다.

---

## 4. 최종 점검 체크리스트
- `/odom` 변위 vs 실제 이동 거리: ±(원하는 오차) 이내
- 제자리 360° 회전: RViz heading 변화 ≈ 360°
- TF 중복 없음, `/tf` 주기가 안정적
- Localization 모드에서 RViz 위치와 실제 위치가 일관

필요 시, 동일 방법으로 **다른 주행 속도/가속 조건**에서도 재검증하세요.
