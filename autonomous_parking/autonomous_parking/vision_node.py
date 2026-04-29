#!/usr/bin/env python3
"""Vision node: slot 유형 및 점유 상태 감지.

파이프라인:
  1. 전방/좌측/우측 camera image 구독
  2. 관측 trigger(/parking/obs_trigger, Bool=True) 수신 시:
       - LiDAR scan으로 각 slot의 점유 여부 판단
       - YOLO model이 있으면 ISA mark(장애인 마크) 감지로 slot 유형 보정
       - model 없으면 YAML 메타데이터 유형을 그대로 사용
  3. /parking/slot_states (std_msgs/String, JSON) 발행

YOLO class:
  0 = 'isa_mark' - 장애인 전용 마크 → slot 유형을 handicapped로 변경
  (점유 여부는 LiDAR로 판단하므로 vehicle class 불필요)

LiDAR 점유 판단:
  관측 포인트에서 각 slot 방향의 LiDAR 거리를 측정한다.
  LiDAR 거리 < slot center까지 거리 - OCCUPANCY_MARGIN이면 차량 있음으로 판단.
"""

import json
import math
import os

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    CV_OK = True
except ImportError:
    CV_OK = False

try:
    from ultralytics import YOLO
    YOLO_OK = True
except ImportError:
    YOLO_OK = False

# LiDAR 거리가 slot center까지 거리보다 이 값 이상 짧으면 점유로 판단
OCCUPANCY_MARGIN = 0.8  # m


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('yolo_model', '')
        model_path = self.get_parameter('yolo_model').value

        pkg = get_package_share_directory('autonomous_parking')
        with open(os.path.join(pkg, 'config', 'slot_metadata.yaml')) as f:
            data = yaml.safe_load(f)
        self.slots = {s['id']: dict(s) for s in data['slots']}

        self.model = None
        if YOLO_OK and model_path and os.path.isfile(model_path):
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLOv8 model 로드 완료: {model_path}')
        else:
            self.get_logger().warn(
                'YOLOv8 model 없음 — slot 유형은 YAML 메타데이터 사용. '
                '점유 여부는 LiDAR로 판단.'
            )

        self.bridge = CvBridge() if CV_OK else None

        # 각 camera의 최신 image (첫 frame 수신 전 None)
        self.images = {
            'front': None,
            'left':  None,
            'right': None,
        }
        self.triggered = False

        # LiDAR / odometry 상태
        self.ranges = None
        self.angle_min = -math.pi
        self.angle_inc = 2 * math.pi / 360
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_ok = False

        # Subscriber
        self.create_subscription(Image, '/ego/camera/image_raw',       self._mk_image_cb('front'), 10)
        self.create_subscription(Image, '/ego/camera_left/image_raw',  self._mk_image_cb('left'),  10)
        self.create_subscription(Image, '/ego/camera_right/image_raw', self._mk_image_cb('right'), 10)
        self.create_subscription(LaserScan, '/ego/scan',  self._scan_cb,  10)
        self.create_subscription(Odometry,  '/ego/odom',  self._odom_cb,  10)
        self.create_subscription(Bool, '/parking/obs_trigger', self._trigger_cb, 10)

        self.pub = self.create_publisher(String, '/parking/slot_states', 10)
        self.create_timer(0.1, self._tick)

        self.get_logger().info('Vision node 시작')

    # ── Callback ──────────────────────────────────────────────────────────

    def _mk_image_cb(self, key):
        def _cb(msg):
            self.images[key] = msg
        return _cb

    def _scan_cb(self, msg):
        self.ranges = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.odom_ok = True

    def _trigger_cb(self, msg):
        if msg.data:
            self.triggered = True

    # ── 메인 tick ──────────────────────────────────────────────────────────

    def _tick(self):
        if not self.triggered:
            return
        self.triggered = False

        available = [k for k, v in self.images.items() if v is not None]
        self.get_logger().info(f'Vision trigger — 사용 가능 camera: {available}')

        # 1단계: LiDAR로 점유 여부 갱신
        self._check_occupancy_lidar()

        # 2단계: YOLO로 ISA mark 감지 → slot 유형 보정
        if self.model is not None and CV_OK and available:
            self._run_yolo(available)
        else:
            self._publish(self.slots)

    # ── LiDAR 점유 감지 ────────────────────────────────────────────────────

    def _lidar_range(self, robot_angle):
        """로봇 frame 기준 각도의 LiDAR 거리값 반환. 유효하지 않으면 inf."""
        if not self.ranges:
            return float('inf')
        idx = round((robot_angle - self.angle_min) / self.angle_inc)
        idx = max(0, min(len(self.ranges) - 1, idx))
        r = self.ranges[idx]
        return float('inf') if (math.isnan(r) or math.isinf(r)) else r

    def _check_occupancy_lidar(self):
        """현재 LiDAR scan과 robot pose로 각 slot의 점유 여부를 판단.

        slot center 방향의 LiDAR 거리가
        center까지 거리 - OCCUPANCY_MARGIN보다 짧으면 차량 있음으로 판단.
        """
        if not self.odom_ok or self.ranges is None:
            self.get_logger().warn('LiDAR 점유 확인 불가 — odometry 또는 scan 미수신')
            return

        for sid, slot in self.slots.items():
            dx = slot['center_x'] - self.robot_x
            dy = slot['center_y'] - self.robot_y
            d_to_center = math.hypot(dx, dy)

            # slot center 방향각을 로봇 frame으로 변환
            angle_world = math.atan2(dy, dx)
            angle_robot = angle_world - self.robot_yaw

            lidar_d = self._lidar_range(angle_robot)

            # center까지 거리보다 OCCUPANCY_MARGIN 이상 짧으면 차량 있음
            occupied = lidar_d < (d_to_center - OCCUPANCY_MARGIN)
            self.slots[sid]['occupied'] = occupied

            self.get_logger().debug(
                f"LiDAR 점유: slot {sid} d_center={d_to_center:.2f}m "
                f"lidar={lidar_d:.2f}m → {'occupied' if occupied else 'empty'}"
            )

    # ── YOLO 추론 (ISA mark 전용) ──────────────────────────────────────────

    def _run_yolo(self, camera_keys):
        """사용 가능한 모든 camera image에 YOLO 추론을 실행해 ISA mark를 감지.

        cls 0 (isa_mark)만 처리한다. 점유 여부는 LiDAR에서 이미 결정됨.
        """
        for key in camera_keys:
            msg = self.images[key]
            if msg is None:
                continue
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().error(f'cv_bridge 오류 [{key}]: {e}')
                continue

            results = self.model.predict(cv_img, conf=0.4, verbose=False)
            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    if cls != 0:   # isa_mark(cls 0) 이외 무시
                        continue

                    cx = float((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                    cy = float((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
                    conf = float(box.conf[0])

                    best_id = self._match_isa_to_slot(cx, cy)
                    if best_id:
                        self.slots[best_id]['type'] = 'handicapped'
                        self.get_logger().info(
                            f"YOLO ISA mark 감지: slot {best_id} → handicapped "
                            f"(conf={conf:.2f}, camera={key})"
                        )

        self._publish(self.slots)

    def _match_isa_to_slot(self, cx, cy):
        """ISA mark bounding box 중심에 가장 가까운 slot ID 반환.

        완전한 구현은 camera intrinsic matrix K와 robot pose를 이용해
        slot 월드 좌표를 image 좌표로 투영해야 한다.
        현재는 robot pose 통합이 없으므로 None 반환 → YAML 유형 유지.
        """
        return None

    # ── Publish ────────────────────────────────────────────────────────────

    def _publish(self, slots):
        payload = [
            {
                'id': sid,
                'type': s['type'],
                'occupied': s['occupied'],
                'center_x': s['center_x'],
                'center_y': s['center_y'],
                'yaw': s['yaw'],
            }
            for sid, s in slots.items()
        ]
        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)
        self.get_logger().info(
            f'slot_states 발행: {sum(1 for s in slots.values() if s["occupied"])}개 점유'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
