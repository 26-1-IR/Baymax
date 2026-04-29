# autonomous_parking 패키지

ROS 2 Humble `ament_python` 기반 자율 주차 패키지입니다. 루트 [README.md](../README.md)에 전체 실행 방법과 노드 설명이 정리되어 있습니다.

## 실제 실행 파일

`setup.py`의 `console_scripts` 기준으로 아래 모듈이 빌드됩니다.

| 실행 이름 | 실제 파일 |
|-----------|-----------|
| `parking_node` | `autonomous_parking/parking_node.py` |
| `vision_node` | `autonomous_parking/vision_node.py` |
| `decision_node` | `autonomous_parking/decision_node.py` |

구버전 `scripts/` 복사본은 제거했습니다.

## 실행

```bash
cd ~/IL
colcon build --symlink-install
source install/setup.bash
ros2 launch autonomous_parking parking_sim.launch.py user_credential:=general
```

월드만 확인할 때:

```bash
ros2 launch autonomous_parking world_only.launch.py
```

## 상태 흐름

```text
INIT → ENTER_LOT → OBS_DRIVE → OBS_WAIT
→ LANE_DRIVE → TURN_TO_SLOT → X_ALIGN → PARK_REVERSE → DONE
```

`OBS_DRIVE` 단계는 관측 지점 도착 즉시 양쪽 카메라 기준으로 슬롯을 스캔합니다. `X_ALIGN` 단계에서 x/yaw 오차를 확인한 뒤, `PARK_REVERSE` 단계에서 `/ego/cmd_vel`의 `linear.x`를 음수로 발행해 후진합니다. 주차 완료는 방지턱 접촉 지점에 도달했을 때만 처리합니다.
