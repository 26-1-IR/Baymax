# ROS 2 기반 자율 주차 시스템

**2391007 인공지능학과 김민정 · 2391017 인공지능학과 이윤지**

주차 슬롯 유형(일반/장애인 전용) 분류와 사용자 자격 기반 필터링을 자율 주행·주차 파이프라인에 통합한 ROS 2 시뮬레이션 프로젝트입니다. 실제 실행되는 노드는 `autonomous_parking/autonomous_parking/*.py`에 있으며, `setup.py`의 `console_scripts`로 빌드됩니다.

> ROS 2 Humble, `ament_python`, Python 노드, `colcon build --symlink-install`, `source install/setup.bash` 흐름을 따릅니다. 현재 시뮬레이션 구현은 `gazebo_ros` 플러그인과 `parking_lot.world`를 사용하는 Gazebo Classic 기반입니다.

---

## 환경

| 항목 | 버전/방식 |
|------|-----------|
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 빌드 시스템 | `ament_python` |
| 시뮬레이터 | Gazebo Classic (`gazebo_ros`) |
| Python | 3.10 |
| 비전 | YOLOv8 선택 사용, 없으면 YAML fallback |

---

## 구조

```text
IL/
├── autonomous_parking/
│   ├── autonomous_parking/
│   │   ├── parking_node.py        # 상태 머신 기반 주차 컨트롤러
│   │   ├── vision_node.py         # 카메라/라이다 기반 슬롯 상태 발행
│   │   └── decision_node.py       # 사용자 자격 기반 슬롯 선택
│   ├── config/
│   │   └── slot_metadata.yaml     # 슬롯 좌표, 유형, ROI, 초기 점유 상태
│   ├── launch/
│   │   ├── parking_sim.launch.py  # Gazebo + 전체 파이프라인
│   │   └── world_only.launch.py   # Gazebo 월드만 실행
│   ├── models/                    # ego_hatchback, 주차 차량, 방지턱 모델
│   ├── worlds/
│   │   └── parking_lot.world
│   ├── media/                     # ISA 마커 텍스처
│   ├── setup.py
│   └── package.xml
├── build/                         # colcon 산출물
├── install/                       # colcon 산출물
└── log/                           # 빌드 로그
```

`scripts/`의 구버전 복사본과 TurtleBot3용 런치 파일은 제거했습니다. 현재 실행 경로는 `ros2 run autonomous_parking <node>` 또는 `parking_sim.launch.py`입니다.

---

## 빌드 및 실행

빌드 후 `install/setup.bash`를 소싱하는 흐름을 기준으로 합니다.

```bash
cd ~/IL
colcon build --symlink-install
source install/setup.bash
```

전체 파이프라인은 아래 명령으로 실행합니다.

```bash
ros2 launch autonomous_parking parking_sim.launch.py
```

실행하면 Gazebo가 바로 뜨기 전에 설정 창이 먼저 열립니다. 여기서 주차 조건을 고를 수 있습니다.

- `일반 슬롯에 주차`: 일반 주차 가능한 빈 슬롯만 대상으로 선택
- `장애인 전용 슬롯 우선`: 빈 장애인 전용 슬롯을 먼저 찾고, 없으면 다른 빈 슬롯 선택
- `YOLOv8 모델 경로`: `.pt` 모델을 사용할 때 입력하거나 파일 선택 버튼으로 지정

YOLO 모델 경로를 비워두면 `slot_metadata.yaml`에 있는 슬롯 정보와 초기 점유 상태를 사용합니다. 설정 창에서 `시작`을 누르면 선택한 조건으로 시뮬레이션이 실행되고, `취소`하거나 창을 닫으면 launch가 중단됩니다.

GUI를 띄우지 않고 바로 실행하고 싶을 때는 `show_config_gui:=false`를 사용합니다.

GUI 없이 일반 슬롯 주차:

```bash
ros2 launch autonomous_parking parking_sim.launch.py \
  show_config_gui:=false \
  user_credential:=general
```

GUI 없이 장애인 전용 슬롯 우선 배정:

```bash
ros2 launch autonomous_parking parking_sim.launch.py \
  show_config_gui:=false \
  user_credential:=handicapped
```

YOLOv8 모델 사용:

```bash
ros2 launch autonomous_parking parking_sim.launch.py \
  yolo_model:=/path/to/best.pt
```

GUI를 켠 상태에서도 위처럼 `yolo_model`을 넘길 수 있고, 이 값은 설정 창의 기본 입력값으로 들어갑니다.

Gazebo 월드만 확인:

```bash
ros2 launch autonomous_parking world_only.launch.py
```

---

## Launch 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `user_credential` | `general` | `general` 또는 `handicapped` |
| `yolo_model` | `""` | YOLOv8 `.pt` 경로. 비어 있으면 YAML 메타데이터 사용 |
| `show_config_gui` | `true` | 실행 전 설정 GUI 표시 여부 |

---

## 노드

### `parking_node`

상태 머신으로 관측, 슬롯 접근, 후방 주차를 수행합니다.

```text
INIT → ENTER_LOT → OBS_DRIVE → OBS_WAIT
→ LANE_DRIVE → TURN_TO_SLOT → X_ALIGN → PARK_REVERSE → DONE
```

| 상태 | 설명 |
|------|------|
| `INIT` | 첫 odometry 수신 대기 |
| `ENTER_LOT` | 진입점 `(-10.5, 0.0)`으로 이동 |
| `OBS_DRIVE` | 관측 포인트로 이동 후 양쪽 카메라로 슬롯 스캔 |
| `OBS_WAIT` | vision/decision 결과 대기 |
| `LANE_DRIVE` | y=0 레인을 따라 목표 슬롯 x좌표까지 이동 |
| `TURN_TO_SLOT` | 후진 주차를 위해 슬롯 반대 방향으로 회전 |
| `X_ALIGN` | 슬롯 중심 x좌표와 진입 yaw가 허용 오차 안인지 확인 |
| `PARK_REVERSE` | `linear.x < 0`로 후진 진입, 방지턱 접촉 지점에서만 주차 완료 |
| `DONE` | 정지 유지 |

| 방향 | 토픽 | 타입 |
|------|------|------|
| Subscribe | `/ego/odom` | `nav_msgs/Odometry` |
| Subscribe | `/ego/scan` | `sensor_msgs/LaserScan` |
| Subscribe | `/parking/target_slot` | `std_msgs/String` |
| Subscribe | `/parking/no_slot` | `std_msgs/Bool` |
| Publish | `/ego/cmd_vel` | `geometry_msgs/Twist` |
| Publish | `/parking/obs_trigger` | `std_msgs/Bool` |

### `vision_node`

관측 트리거를 받으면 슬롯 상태를 발행합니다. YOLOv8 모델이 있으면 LiDAR로 점유 여부를 갱신하고 ISA 마크를 감지하며, 모델이 없으면 `slot_metadata.yaml`의 슬롯 유형과 점유 상태를 그대로 사용합니다.

| 방향 | 토픽 | 타입 |
|------|------|------|
| Subscribe | `/ego/camera/image_raw` | `sensor_msgs/Image` |
| Subscribe | `/ego/camera_left/image_raw` | `sensor_msgs/Image` |
| Subscribe | `/ego/camera_right/image_raw` | `sensor_msgs/Image` |
| Subscribe | `/ego/scan` | `sensor_msgs/LaserScan` |
| Subscribe | `/ego/odom` | `nav_msgs/Odometry` |
| Subscribe | `/parking/obs_trigger` | `std_msgs/Bool` |
| Publish | `/parking/slot_states` | `std_msgs/String` JSON |

### `decision_node`

`/parking/slot_states`를 받아 사용자 자격에 맞는 빈 슬롯을 선택합니다.

| 방향 | 토픽 | 타입 |
|------|------|------|
| Subscribe | `/parking/slot_states` | `std_msgs/String` JSON |
| Publish | `/parking/target_slot` | `std_msgs/String` |
| Publish | `/parking/no_slot` | `std_msgs/Bool` |

선택 순서:

1. 점유 슬롯 제외
2. `general` 사용자는 장애인 전용 슬롯 제외
3. `handicapped` 사용자는 장애인 전용 슬롯 우선
4. 진입점 `x=-10.5`와 가까운 슬롯 선택

---

## 차량 및 센서

`ego_hatchback`은 `/ego/cmd_vel`을 받는 planar 이동 차량 모델입니다. 후진 주차 단계에서는 `parking_node`가 `linear.x < 0`을 발행하고, Gazebo 플러그인이 이를 차량 기준 후진 속도로 적용합니다.

| 센서 | 위치 | 토픽 |
|------|------|------|
| 2D LiDAR | 지붕, 360도 | `/ego/scan` |
| 전방 카메라 | 전방, 하향 틸트 | `/ego/camera/image_raw` |
| 좌측 카메라 | 좌측면 | `/ego/camera_left/image_raw` |
| 우측 카메라 | 우측면 | `/ego/camera_right/image_raw` |
| IMU | 차체 중앙 | `/ego/imu` |

---

## 슬롯 구성

- 총 16개 슬롯: A1-A8, B1-B8
- 장애인 전용: A1, A2, B1, B2
- 일반: A3-A8, B3-B8
- 초기 점유: A4, A7, B3, B6

```text
     A1   A2   A3   A4   A5   A6   A7   A8
     HC   HC   GN  [GN]  GN   GN  [GN]  GN
 ←━━━━━━━━━━━━━━━ 레인 (y=0) ━━━━━━━━━━━━━━━→
     HC   HC  [GN]  GN   GN  [GN]  GN   GN
     B1   B2   B3   B4   B5   B6   B7   B8
```

---

## 자주 쓰는 명령

```bash
cd ~/IL
colcon build --symlink-install
source install/setup.bash
ros2 launch autonomous_parking parking_sim.launch.py user_credential:=general
```

```bash
ros2 topic echo /ego/cmd_vel
ros2 topic echo /ego/odom
ros2 topic echo /parking/target_slot
```

```bash
pkill -f gazebo
pkill -f gzserver
pkill -f gzclient
```

---

## 작성자

- Yoonji Lee
- Minjeong Kim
