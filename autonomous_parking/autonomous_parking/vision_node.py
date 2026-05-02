
import json
import math
import os

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

try:
    from ultralytics import YOLO
    YOLO_OK = True
    YOLO_IMPORT_ERROR = ''
except ImportError as e:
    YOLO_OK = False
    YOLO_IMPORT_ERROR = str(e)

OCCUPANCY_MARGIN = 0.8  # m


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('yolo_model', '')
        self.declare_parameter('use_lidar_occupancy', False)

        model_path = self.get_parameter('yolo_model').value
        self.use_lidar_occupancy = bool(self.get_parameter('use_lidar_occupancy').value)

        pkg = get_package_share_directory('autonomous_parking')
        with open(os.path.join(pkg, 'config', 'slot_metadata.yaml')) as f:
            data = yaml.safe_load(f)

        self.base_slots = {s['id']: dict(s) for s in data['slots']}
        self.slots = {sid: dict(s) for sid, s in self.base_slots.items()}

        self.slot_rois = {
            'front': {
                'A1': (60, 60, 240, 210),
                'A2': (240, 60, 470, 210),
                'B1': (0, 220, 220, 360),
                'B2': (220, 220, 470, 360),
            },
            'left': {
                'A1': (0, 60, 180, 220),
                'A2': (180, 60, 420, 220),
                'B1': (0, 220, 150, 360),
                'B2': (150, 220, 320, 360),
            },
            'right': {
                'A1': (320, 120, 470, 250),
                'A2': (470, 120, 640, 250),
                'B1': (340, 220, 510, 340),
                'B2': (510, 220, 640, 340),
            },
        }

        self.model = None
        self.get_logger().info(f'yolo_model parameter: "{model_path}"')
        self.get_logger().info(f'YOLO_OK: {YOLO_OK}')
        self.get_logger().info(f'use_lidar_occupancy: {self.use_lidar_occupancy}')

        if not YOLO_OK:
            self.get_logger().warn(f'ultralytics import 실패: {YOLO_IMPORT_ERROR}')
        elif not model_path:
            self.get_logger().warn('yolo_model 파라미터가 비어 있음')
        elif not os.path.isfile(model_path):
            self.get_logger().warn(f'모델 파일이 존재하지 않음: {model_path}')
        else:
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(f'YOLOv8 model 로드 완료: {model_path}')
            except Exception as e:
                self.get_logger().error(f'YOLO model 로드 실패: {e}')
                self.model = None

        self.images = {
            'front': None,
            'left': None,
            'right': None,
        }
        self.triggered = False

        self.ranges = None
        self.angle_min = -math.pi
        self.angle_inc = 2 * math.pi / 360.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_ok = False

        self.create_subscription(Image, '/ego/camera/image_raw', self._mk_image_cb('front'), 10)
        self.create_subscription(Image, '/ego/camera_left/image_raw', self._mk_image_cb('left'), 10)
        self.create_subscription(Image, '/ego/camera_right/image_raw', self._mk_image_cb('right'), 10)
        self.create_subscription(LaserScan, '/ego/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/ego/odom', self._odom_cb, 10)
        self.create_subscription(Bool, '/parking/obs_trigger', self._trigger_cb, 10)

        self.pub = self.create_publisher(String, '/parking/slot_states', 10)
        self.create_timer(0.1, self._tick)

        self.get_logger().info('Vision node 시작')

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

    def _tick(self):
        if not self.triggered:
            return
        self.triggered = False

        # 매 관측마다 YAML 기본 상태로 초기화
        self.slots = {sid: dict(s) for sid, s in self.base_slots.items()}

        available = [k for k, v in self.images.items() if v is not None]
        self.get_logger().info(f'Vision trigger — 사용 가능 camera: {available}')

        # occupancy는 기본적으로 YAML 사용
        if self.use_lidar_occupancy:
            self._check_occupancy_lidar()
        else:
            self.get_logger().debug('LiDAR occupancy 비활성화 — YAML occupied 사용')

        if self.model is None or not available:
            self._publish(self.slots)
            return

        self._run_yolo(available)

    def _image_msg_to_bgr(self, msg: Image):
        enc = msg.encoding.lower()

        if enc in ('bgr8', 'rgb8'):
            channels = 3
        elif enc in ('bgra8', 'rgba8'):
            channels = 4
        elif enc in ('mono8', '8uc1'):
            channels = 1
        else:
            raise ValueError(f'지원하지 않는 image encoding: {msg.encoding}')

        raw = np.frombuffer(msg.data, dtype=np.uint8)

        row_stride = msg.step
        expected = msg.height * row_stride
        if raw.size < expected:
            raise ValueError(f'이미지 버퍼 크기 부족: got={raw.size}, expected>={expected}')

        raw = raw[:expected].reshape((msg.height, row_stride))
        pixel_bytes = msg.width * channels
        raw = raw[:, :pixel_bytes]

        if channels == 1:
            img = raw.reshape((msg.height, msg.width))
            return np.stack([img, img, img], axis=-1)

        img = raw.reshape((msg.height, msg.width, channels))

        if enc == 'bgr8':
            return img.copy()
        if enc == 'rgb8':
            return img[:, :, ::-1].copy()
        if enc == 'bgra8':
            return img[:, :, :3].copy()
        if enc == 'rgba8':
            return img[:, :, [2, 1, 0]].copy()

        raise ValueError(f'처리하지 못한 encoding: {msg.encoding}')

    def _lidar_range(self, robot_angle):
        if not self.ranges:
            return float('inf')
        idx = round((robot_angle - self.angle_min) / self.angle_inc)
        idx = max(0, min(len(self.ranges) - 1, idx))
        r = self.ranges[idx]
        return float('inf') if (math.isnan(r) or math.isinf(r)) else r

    def _check_occupancy_lidar(self):
        if not self.odom_ok or self.ranges is None:
            return

        for sid, slot in self.slots.items():
            dx = slot['center_x'] - self.robot_x
            dy = slot['center_y'] - self.robot_y
            d_to_center = math.hypot(dx, dy)

            angle_world = math.atan2(dy, dx)
            angle_robot = angle_world - self.robot_yaw
            lidar_d = self._lidar_range(angle_robot)

            occupied = lidar_d < (d_to_center - OCCUPANCY_MARGIN)
            self.slots[sid]['occupied'] = occupied

    def _run_yolo(self, camera_keys):
        for key in camera_keys:
            msg = self.images[key]
            if msg is None:
                continue

            try:
                bgr = self._image_msg_to_bgr(msg)
                results = self.model.predict(source=bgr, conf=0.4, verbose=False)
            except Exception as e:
                self.get_logger().error(f'YOLO 처리 실패 [{key}]: {e}')
                continue

            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    if cls != 0:
                        continue

                    cx = float((box.xyxy[0][0] + box.xyxy[0][2]) / 2.0)
                    cy = float((box.xyxy[0][1] + box.xyxy[0][3]) / 2.0)
                    conf = float(box.conf[0])

                    slot_id = self._match_isa_to_slot(key, cx, cy)
                    if slot_id is None:
                        self.get_logger().info(
                            f'ISA mark 미매칭: camera={key}, conf={conf:.2f}, cx={cx:.1f}, cy={cy:.1f}'
                        )
                        continue

                    self.slots[slot_id]['type'] = 'handicapped'
                    self.get_logger().info(
                        f'YOLO ISA mark 감지: slot {slot_id} → handicapped '
                        f'(camera={key}, conf={conf:.2f}, cx={cx:.1f}, cy={cy:.1f})'
                    )

        self._publish(self.slots)

    def _match_isa_to_slot(self, camera_key, cx, cy):
        candidates = self.slot_rois.get(camera_key, {})
        matched = []

        for slot_id, (x1, y1, x2, y2) in candidates.items():
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                roi_cx = (x1 + x2) / 2.0
                roi_cy = (y1 + y2) / 2.0
                dist = math.hypot(cx - roi_cx, cy - roi_cy)
                matched.append((dist, slot_id))

        if not matched:
            return None

        matched.sort(key=lambda x: x[0])
        return matched[0][1]

    def _publish(self, slots):
        occupied_ids = [sid for sid, s in slots.items() if s['occupied']]
        self.get_logger().info(f'occupied slots: {occupied_ids}')

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()