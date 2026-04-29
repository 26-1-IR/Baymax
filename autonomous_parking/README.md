# 자율 주차 시스템 (Autonomous Parking System)

## 개요

ROS 2 Humble과 Gazebo Fortress 기반의 자율 주차 시뮬레이션 시스템입니다. 로봇이 주차장 내에서 자동으로 주차 슬롯을 찾아 주차하는 전체 파이프라인을 구현합니다.

## 시스템 구조

```
autonomous_parking/
├── autonomous_parking/            # Python 패키지
│   ├── __init__.py
│   ├── parking_node.py           # 주차 제어 노드 (상태 머신)
│   ├── vision_node.py            # 비전 및 감지 노드 (YOLO)
│   └── decision_node.py          # 슬롯 선택 의사 결정 노드
├── launch/                        # Launch 파일
├── config/                        # YAML 설정 파일
│   └── slot_metadata.yaml        # 슬롯 메타데이터
├── models/                        # Gazebo 모델 (차량, 방지턱 등)
├── worlds/                        # Gazebo 월드 파일
├── media/                         # 텍스처, 마크 이미지
├── maps/                          # SLAM 맵 파일
├── resource/                      # 패키지 리소스
├── setup.py                       # setuptools 설정
├── setup.cfg                      # setuptools 설정
└── package.xml                    # ROS 2 패키지 정의
```

## 주요 노드

### 1. parking_node (주차 제어)
- **역할**: 상태 머신 기반 자율 주차 제어
- **상태**:
  - `INIT`: 초기화
  - `ENTER_LOT`: 주차장 진입
  - `OBS_DRIVE`: 관측 포인트로 이동
  - `OBS_YAW`: 관측 방향으로 회전
  - `OBS_WAIT`: vision_node 스캔 대기
  - `LANE_DRIVE`: 차선을 따라 슬롯 위치로 이동
  - `TURN_TO_SLOT`: 슬롯 방향으로 회전
  - `X_ALIGN`: 슬롯 x 중심 정렬
  - `PARK_FORWARD`: 후진으로 슬롯 진입
  - `DONE`: 주차 완료

**구독 토픽**:
- `/ego/odom` (Odometry) - 위치/방향 정보
- `/ego/scan` (LaserScan) - LiDAR 거리 센서
- `/parking/target_slot` (String) - decision_node가 선택한 슬롯 ID
- `/parking/no_slot` (Bool) - 가능한 슬롯 없음 신호

**발행 토픽**:
- `/ego/cmd_vel` (Twist) - 속도 명령
- `/parking/obs_trigger` (Bool) - vision_node 스캔 트리거

### 2. vision_node (비전 감지)
- **역할**: 카메라 이미지에서 슬롯 상태 감지 (YOLOv8)
- **카메라**: 전방, 좌측, 우측 3개 카메라 사용

**구독 토픽**:
- `/ego/camera/image_raw` (Image) - 전방 카메라
- `/ego/camera_left/image_raw` (Image) - 좌측 카메라
- `/ego/camera_right/image_raw` (Image) - 우측 카메라
- `/parking/obs_trigger` (Bool) - 스캔 트리거

**발행 토픽**:
- `/parking/slot_states` (String) - 슬롯 상태 JSON

**YOLO 클래스**:
- `0`: `isa_mark` - 장애인 전용 슬롯 마크
- `1`: `vehicle` - 주차된 차량

### 3. decision_node (의사 결정)
- **역할**: 사용자 자격에 따라 최적 슬롯 선택

**구독 토픽**:
- `/parking/slot_states` (String) - vision_node의 슬롯 상태

**발행 토픽**:
- `/parking/target_slot` (String) - 선택된 슬롯 ID
- `/parking/no_slot` (Bool) - 유효한 슬롯 없음

**파라미터**:
- `user_credential` (string, 기본값: 'general')
  - `'general'`: 일반 사용자 (장애인 슬롯 제외)
  - `'handicapped'`: 장애인 (모든 빈 슬롯 사용 가능, 장애인 슬롯 우선)

## 설치 및 실행

### 1. 필수 패키지 설치

```bash
sudo apt-get install -y ros-humble-gazebo-ros \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-plugins \
    ros-humble-teleop-twist-keyboard
```

YOLOv8 사용 시:
```bash
pip install ultralytics opencv-python
```

### 2. 빌드

```bash
cd ~/IL  # 또는 해당 워크스페이스
colcon build --symlink-install
source install/setup.bash
```

### 3. 실행

**전체 시뮬레이션 실행**:
```bash
ros2 launch autonomous_parking parking_sim.launch.py user_credential:=general
```

**개별 노드 실행**:
```bash
# 터미널 1: Gazebo + 주차 노드
ros2 launch autonomous_parking parking_sim.launch.py

# 터미널 2: vision_node
ros2 run autonomous_parking vision_node

# 터미널 3: decision_node
ros2 run autonomous_parking decision_node --ros-args -p user_credential:=general
```

**ROS 2 CLI로 직접 실행**:
```bash
ros2 run autonomous_parking parking_node
ros2 run autonomous_parking vision_node
ros2 run autonomous_parking decision_node
```

## 주요 기능

### 자율 주행
- LiDAR 기반 거리 센서로 방지턱 감지
- 상태 머신 기반 안정적인 제어
- 좌우 방향 정렬 유지

### 비전 기반 감지
- 3개 카메라(전방, 좌, 우)로 360° 커버리지
- YOLOv8 기반 객체 감지
- 장애인 슬롯 마크 인식

### 의사 결정
- 사용자 자격별 슬롯 필터링
- 거리 기준 최적화
- 동적 재계획 (충돌 시)

## 설정 파일

### slot_metadata.yaml
- 슬롯 위치, 타입(일반/장애인), ROI 정보
- YOLO 감지 결과를 슬롯과 매핑하는 데 사용

### launch 파일
- `parking_sim.launch.py`: 전체 시뮬레이션 실행
- `parking_world.launch.py`: 월드만 실행
- `world_only.launch.py`: Gazebo만 실행

## 의존성

- **ROS 2 Humble**
- **Gazebo Fortress**
- **Python 3.10+**
- **OpenCV** (cv_bridge)
- **PyYAML**
- **Ultralytics YOLO** (YOLOv8, 옵션)

## 개발 환경

- **ROS 2 버전**: Humble
- **Gazebo 버전**: Fortress
- **빌드 시스템**: ament_python
- **Python 패키지 포맷**: setuptools

## 문제 해결

### 빌드 오류
```bash
# 완전 재빌드
rm -rf build install log
colcon build --symlink-install
```

### 토픽이 보이지 않음
```bash
# ROS_DOMAIN_ID 확인 (학번 끝 3자리 권장)
export ROS_DOMAIN_ID=<학번_끝_3자리>
```

### Gazebo 시각화 문제
```bash
# GPU 가속 비활성화 (필요시)
export LIBGL_ALWAYS_SOFTWARE=1
```

## 라이센스

Apache-2.0

## 작성자

Yoonji Lee (estelle0329@ewha.ac.kr)
Minjeong Kim (laputais@ewhain.net)