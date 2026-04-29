#!/usr/bin/env python3
"""자율 주차 제어 node (ego_hatchback용).

State machine:
  INIT         → 첫 odometry 대기
  ENTER_LOT    → 주차장 진입점(-10.5, 0.0)으로 이동
  OBS_DRIVE    → 관측 포인트로 이동 후 vision_node scan trigger
  OBS_WAIT     → decision_node 목표 slot 대기
  LANE_DRIVE   → y=0 차선을 따라 slot x 위치로 이동
  TURN_TO_SLOT → slot 방향(북/남)으로 회전
  X_ALIGN      → 회전 후 slot x 중심 정렬
  PARK_REVERSE → slot으로 후진; 깊이+ROI 조건 또는 장애물 감지 시 정지
  DONE         → 위치 유지

발행 topic:
  /parking/obs_trigger  (Bool)   - 각 관측점에서 vision_node scan 실행

구독 topic:
  /parking/target_slot  (String) - decision_node가 선택한 slot ID
  /parking/no_slot      (Bool)   - 가능한 slot 없음 신호
"""

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from ament_index_python.packages import get_package_share_directory

try:
    from gazebo_msgs.srv import SpawnEntity, DeleteEntity
    GAZEBO_SRV_OK = True
except ImportError:
    GAZEBO_SRV_OK = False


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# slot_metadata.yaml 기준 관측 포인트
OBS_POINTS = [
    {'id': 'obs_1', 'x': -5.0, 'y': 0.0},
    {'id': 'obs_2', 'x':  5.0, 'y': 0.0},
]


class ParkingNode(Node):
    # Gazebo 충돌 기준으로 wheel_stopper에 닿았을 때의 차량 중심 y좌표.
    # 후방 LiDAR는 y=5.5대에서도 방지턱을 보기 때문에, 완료는 y 기준을 함께 본다.
    PARK_Y_A =  5.85  # Row A — 북쪽 방지턱 접촉
    PARK_Y_B = -5.85  # Row B — 남쪽 방지턱 접촉

    PARK_SPEED = 0.35   # 후진 주차 속도 (m/s)
    LANE_SPEED = 1.5    # 레인 주행 속도 (m/s)

    # 후방 LiDAR는 차체 중심 높이에서 벽/방지턱을 일찍 볼 수 있으므로
    # 단독 완료 조건으로 쓰지 않고, 접촉 y 근처에서만 보조 확인에 사용한다.
    STOPPER_LIDAR_DIST = 2.12  # 방지턱 근접 판정 거리 (m)
    REAR_EMRG_DIST     = 0.3   # 비상 후방 감지 거리 (m)
    SIDE_STOP_DIST     = 0.3   # 측면 비상 장애물 감지 거리 (m)
    X_ALIGN_TOL        = 0.25  # LANE_DRIVE 이후 허용 x 정렬 오차 (m)
    STOPPER_ZONE_MARGIN = 0.08 # 접촉 지점 근처에서만 LiDAR 완료 판정 허용 (m)

    OBS_WAIT_TIMEOUT = 10.0   # slot 선택 대기 timeout (초)

    def __init__(self):
        super().__init__('parking_node')

        pkg = get_package_share_directory('autonomous_parking')
        with open(os.path.join(pkg, 'config', 'slot_metadata.yaml')) as f:
            data = yaml.safe_load(f)
        self.slots = {s['id']: s for s in data['slots']}

        self.x = -12.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ok = False

        self.ranges = None
        self.angle_min = -math.pi
        self.angle_inc = 2 * math.pi / 360

        # 상태 변수
        self.state = 'INIT'
        self.target_slot = None
        self.slot_target_yaw = 0.0
        self.obs_idx = 0          # OBS_POINTS 인덱스
        self.obs_wait_start = None
        self.received_target = False
        self._last_cmd = (0.0, 0.0, 0.0)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/ego/cmd_vel', 10)
        self.obs_trigger_pub = self.create_publisher(Bool, '/parking/obs_trigger', 10)

        # Subscriber
        self.create_subscription(Odometry, '/ego/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/ego/scan', self._scan_cb, 10)
        self.create_subscription(String, '/parking/target_slot', self._target_cb, 10)
        self.create_subscription(Bool, '/parking/no_slot', self._no_slot_cb, 10)

        # Gazebo slot highlight
        self._highlight_name = None
        if GAZEBO_SRV_OK:
            self._spawn_client  = self.create_client(SpawnEntity,  '/spawn_entity')
            self._delete_client = self.create_client(DeleteEntity, '/delete_entity')
        else:
            self._spawn_client  = None
            self._delete_client = None
            self.get_logger().warn('gazebo_msgs 없음 — slot highlight 비활성화')

        self.create_timer(0.05, self._tick)
        self.create_timer(0.5, self._log_status)   # 2 Hz 상태 로그
        self.get_logger().info('Parking node 시작')

    # ── Callback ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.odom_ok = True

    def _scan_cb(self, msg):
        self.ranges = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    # LANE_DRIVE 이후에는 이미 주행 중인 slot을 덮어쓰지 않도록
    # decision_node의 target slot을 수신할 수 있는 state 목록
    _RECEPTIVE_STATES = frozenset({
        'INIT', 'ENTER_LOT', 'OBS_DRIVE', 'OBS_WAIT'
    })

    def _target_cb(self, msg):
        if self.state not in self._RECEPTIVE_STATES:
            self.get_logger().debug(
                f'_target_cb: state {self.state}에서 뒤늦은 target 무시'
            )
            return
        slot_id = msg.data.strip()
        if slot_id not in self.slots:
            self.get_logger().error(f'알 수 없는 slot ID: {slot_id}')
            return
        self.get_logger().info(f'decision_node 선택 slot: {slot_id}')
        self._set_target(self.slots[slot_id])
        self.received_target = True

    def _no_slot_cb(self, msg):
        if msg.data:
            self.get_logger().error('유효한 slot 없음 — DONE으로 전환')
            self._pub(0.0, 0.0)
            self.state = 'DONE'

    # ── 헬퍼 ──────────────────────────────────────────────────────────────

    def _lidar_range(self, robot_angle):
        """로봇 frame 기준 각도의 LiDAR 거리값 반환. 유효하지 않으면 inf."""
        if not self.ranges:
            return float('inf')
        idx = round((robot_angle - self.angle_min) / self.angle_inc)
        idx = max(0, min(len(self.ranges) - 1, idx))
        r = self.ranges[idx]
        return float('inf') if (math.isnan(r) or math.isinf(r)) else r

    def _stopper_reached(self):
        """후방 LiDAR가 접촉 거리에서 방지턱을 감지하면 True."""
        for deg in range(-6, 7, 2):
            r = self._lidar_range(math.pi + math.radians(deg))
            if r < self.STOPPER_LIDAR_DIST:
                return True
        return False

    def _stopper_contact_y_reached(self):
        """차량 중심이 방지턱 접촉 y좌표에 도달했으면 True."""
        if not self.target_slot:
            return False
        row = self.target_slot['id'][0]
        return self.y >= self.PARK_Y_A if row == 'A' else self.y <= self.PARK_Y_B

    def _near_stopper_contact_zone(self):
        """LiDAR 접촉 완료를 믿을 수 있는 방지턱 근처 영역."""
        if not self.target_slot:
            return False
        row = self.target_slot['id'][0]
        if row == 'A':
            return abs(self.y - self.PARK_Y_A) <= self.STOPPER_ZONE_MARGIN
        return abs(self.y - self.PARK_Y_B) <= self.STOPPER_ZONE_MARGIN

    def _stopper_check_enabled(self):
        """차량 중심이 slot 입구 근처를 지난 뒤에만 방지턱 판정을 허용."""
        if not self.target_slot:
            return False
        roi = self.target_slot.get('roi')
        if not roi:
            return False
        row = self.target_slot['id'][0]
        margin = 0.3
        if row == 'A':
            return self.y >= roi['y_min'] - margin
        return self.y <= roi['y_max'] + margin

    def _side_check_enabled(self):
        """slot 안쪽에 진입한 뒤에만 측면 장애물 판정을 허용."""
        return self._stopper_check_enabled()

    def _rear_emergency(self):
        """후방에 REAR_EMRG_DIST 이하의 예상치 못한 장애물이 있으면 True."""
        for deg in range(-6, 7, 2):
            if self._lidar_range(math.pi + math.radians(deg)) < self.REAR_EMRG_DIST:
                return True
        return False

    def _sides_clear(self):
        """후진 주차 중 양 측면에 SIDE_STOP_DIST 이하 장애물이 없으면 True.

        차가 남향(-π/2)일 때 slot 좌우는 동/서 방향(로봇 frame ±90°).
        실제 측면 방향 감지를 위해 ±80-100° 범위를 사용한다.
        """
        for deg in range(80, 101, 5):
            if self._lidar_range(math.radians(deg)) < self.SIDE_STOP_DIST:
                return False
            if self._lidar_range(math.radians(-deg)) < self.SIDE_STOP_DIST:
                return False
        return True

    def _in_slot_boundary(self):
        """차량 중심이 목표 slot의 ROI 내부에 있으면 True."""
        roi = self.target_slot.get('roi')
        if not roi:
            return False
        return (roi['x_min'] < self.x < roi['x_max'] and
                roi['y_min'] < self.y < roi['y_max'])

    def _complete_parking(self, reason):
        sx = self.target_slot['center_x']
        rear_dist = self._lidar_range(math.pi)
        self._pub(0.0, 0.0)
        self.get_logger().info(
            f"주차 완료: {self.target_slot['id']} [{reason}] "
            f"pos=({self.x:.2f},{self.y:.2f}) "
            f"x_err={self.x - sx:.3f}m "
            f"rear={rear_dist:.2f}m"
        )
        self._highlight_slot(self.target_slot, color='blue')
        self.state = 'DONE'

    # ── Gazebo slot highlight ──────────────────────────────────────────────

    def _highlight_slot(self, slot, color='green'):
        """목표 slot 위에 반투명 색상 박스를 Gazebo에 spawn."""
        if not self._spawn_client:
            return
        self._delete_highlight()

        r, g, b = (0.0, 1.0, 0.0) if color == 'green' else (0.2, 0.5, 1.0)
        name = 'slot_highlight'
        sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="floor">
        <pose>0 0 0.005 0 0 0</pose>
        <geometry><box><size>2.6 4.3 0.008</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 0.6</ambient>
          <diffuse>{r} {g} {b} 0.6</diffuse>
          <emissive>{r*0.4:.2f} {g*0.4:.2f} {b*0.4:.2f} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf
        req.initial_pose = Pose()
        req.initial_pose.position.x = float(slot['center_x'])
        req.initial_pose.position.y = float(slot['center_y'])
        req.initial_pose.position.z = 0.0
        self._spawn_client.call_async(req)
        self._highlight_name = name
        self.get_logger().info(
            f"Slot highlight spawn: ({slot['center_x']}, {slot['center_y']}) [{color}]"
        )

    def _delete_highlight(self):
        if not self._delete_client or not self._highlight_name:
            return
        req = DeleteEntity.Request()
        req.name = self._highlight_name
        self._delete_client.call_async(req)
        self._highlight_name = None

    def _goto(self, tx, ty, tol=0.35, speed=None):
        """목표 지점(tx, ty)으로 이동. 도달하면 (0, 0, True) 반환."""
        if speed is None:
            speed = self.LANE_SPEED
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if dist < tol:
            return 0.0, 0.0, True
        target_yaw = math.atan2(dy, dx)
        err = angle_diff(target_yaw, self.yaw)
        lin = min(speed, 1.5 * dist)
        ang = max(-1.5, min(1.5, 2.0 * err))
        if abs(err) > 0.5:
            lin *= 0.15
        elif abs(err) > 0.2:
            lin *= 0.5
        return lin, ang, False

    def _pub(self, lin, ang, lat=0.0):
        t = Twist()
        t.linear.x = float(lin)
        t.linear.y = float(lat)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)
        self._last_cmd = (lin, lat, ang)

    def _log_status(self):
        if not self.odom_ok:
            return
        fwd  = self._lidar_range(0.0)
        left = self._lidar_range(math.pi / 2)
        right= self._lidar_range(-math.pi / 2)
        rear = self._lidar_range(math.pi)
        last_cmd = getattr(self, '_last_cmd', (0.0, 0.0, 0.0))
        if len(last_cmd) == 2:
            lin, lat, ang = last_cmd[0], 0.0, last_cmd[1]
        else:
            lin, lat, ang = last_cmd

        target_info = ''
        if self.target_slot:
            sx = self.target_slot['center_x']
            sy = self.target_slot['center_y']
            dist = math.hypot(sx - self.x, sy - self.y)
            target_info = (
                f" | target={self.target_slot['id']}"
                f" ({sx:.1f},{sy:.1f}) dist={dist:.2f}m"
            )

        self.get_logger().info(
            f"[{self.state}] "
            f"pos=({self.x:.2f},{self.y:.2f}) yaw={math.degrees(self.yaw):.1f}deg "
            f"cmd=lin:{lin:.2f} lat:{lat:.2f} ang:{ang:.2f} "
            f"lidar F:{fwd:.2f} L:{left:.2f} R:{right:.2f} B:{rear:.2f}"
            + target_info
        )

    def _fire_obs_trigger(self):
        msg = Bool()
        msg.data = True
        self.obs_trigger_pub.publish(msg)

    # ── State machine ──────────────────────────────────────────────────────

    def _tick(self):
        if not self.odom_ok:
            return

        # 멈춤 감지: 4초 이상 0.05m 미만 이동 시 DONE 전환
        if self.state not in ('INIT', 'OBS_WAIT', 'DONE'):
            if math.hypot(self.x - self._stuck_last_pos[0],
                        self.y - self._stuck_last_pos[1]) > 0.05:
                self._stuck_last_pos = (self.x, self.y)
                self._stuck_since = self.get_clock().now()
            elif (self.get_clock().now() - self._stuck_since).nanoseconds / 1e9 > 4.0:
                if (
                    self.state == 'PARK_REVERSE' and
                    self.target_slot and
                    self._in_slot_boundary() and
                    self._near_stopper_contact_zone()
                ):
                    self._complete_parking('방지턱 접촉 정지')
                    return
                self.get_logger().error(
                    f'멈춤 감지: ({self.x:.2f},{self.y:.2f}) state={self.state} — DONE'
                )
                self._pub(0.0, 0.0)
                self.state = 'DONE'
                return

        # ── INIT ──────────────────────────────────────────────────────────
        if self.state == 'INIT':
            self.get_logger().info('주차장 진입 시작')
            self.state = 'ENTER_LOT'
            self._stuck_last_pos = (self.x, self.y)
            self._stuck_since = self.get_clock().now()

        # ── ENTER_LOT ─────────────────────────────────────────────────────
        elif self.state == 'ENTER_LOT':
            lin, ang, done = self._goto(-10.5, 0.0)
            if done:
                self.get_logger().info(
                    f'ENTER_LOT 완료 — pos=({self.x:.2f},{self.y:.2f})'
                )
                self._pub(0.0, 0.0)
                self.obs_idx = 0
                self.state = 'OBS_DRIVE'
            else:
                self._pub(lin, ang)

        # ── OBS_DRIVE ─────────────────────────────────────────────────────
        # 양쪽 카메라가 슬롯 열을 커버하므로 관측 yaw 회전 없이 바로 스캔한다.
        elif self.state == 'OBS_DRIVE':
            obs = OBS_POINTS[self.obs_idx]
            lin, ang, done = self._goto(obs['x'], obs['y'], tol=0.6)
            if done:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f"OBS_DRIVE 완료 — pos=({self.x:.2f},{self.y:.2f}) "
                    "— vision scan trigger"
                )
                self._fire_obs_trigger()
                self.obs_wait_start = self.get_clock().now()
                self.received_target = False
                self.state = 'OBS_WAIT'
            else:
                self._pub(lin, ang)

        # ── OBS_WAIT ──────────────────────────────────────────────────────
        # decision_node의 slot 선택 결과 대기.
        elif self.state == 'OBS_WAIT':
            self._pub(0.0, 0.0)

            if self.received_target:
                self.state = 'LANE_DRIVE'
                self._stuck_last_pos = (self.x, self.y)
                self._stuck_since = self.get_clock().now()
                return

            elapsed = (self.get_clock().now() - self.obs_wait_start).nanoseconds / 1e9
            if elapsed > self.OBS_WAIT_TIMEOUT:
                self.get_logger().warn(
                    f'{OBS_POINTS[self.obs_idx]["id"]} timeout — 다음 관측 포인트로 이동'
                )
                self.obs_idx += 1
                if self.obs_idx >= len(OBS_POINTS):
                    self.get_logger().error('모든 관측 포인트 소진 — DONE')
                    self.state = 'DONE'
                else:
                    self.state = 'OBS_DRIVE'

        # ── LANE_DRIVE ────────────────────────────────────────────────────
        # 2단계: (A) 목표 방향으로 제자리 회전, (B) 조향 보정하며 주행.
        # _goto 단독 사용 시 시작 yaw가 90° 이상 어긋나면 나선형 궤적 발생.
        elif self.state == 'LANE_DRIVE':
            sx = self.target_slot['center_x']
            dx = sx - self.x
            dy = 0.0 - self.y
            dist = math.hypot(dx, dy)

            if dist < 0.15:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f"LANE_DRIVE 완료 — pos=({self.x:.2f},{self.y:.2f}) "
                    f"target_x={sx:.2f} x_err={dx:.3f}m"
                )
                self.state = 'TURN_TO_SLOT'
                self._stuck_last_pos = (self.x, self.y)
                self._stuck_since = self.get_clock().now()
                return

            target_bearing = math.atan2(dy, dx)
            yaw_err = angle_diff(target_bearing, self.yaw)

            # 단계 A: 목표 방향과의 오차가 클 때 제자리 회전
            if abs(yaw_err) > 0.15:
                ang = max(-1.2, min(1.2, 2.0 * yaw_err))
                self._pub(0.0, ang)
                self.get_logger().debug(
                    f"LANE_YAW: bearing={math.degrees(target_bearing):.1f}deg "
                    f"err={math.degrees(yaw_err):.1f}deg"
                )
                return

            # 단계 B: 조향 보정을 적용하며 주행
            lin = min(self.LANE_SPEED, 1.5 * dist)
            ang = max(-0.8, min(0.8, 2.0 * yaw_err))
            self._pub(lin, ang)

        # ── TURN_TO_SLOT ──────────────────────────────────────────────────
        elif self.state == 'TURN_TO_SLOT':
            err = angle_diff(self.slot_target_yaw, self.yaw)
            if abs(err) < 0.05:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f"TURN 완료 — yaw={math.degrees(self.yaw):.1f}deg "
                    f"target={math.degrees(self.slot_target_yaw):.1f}deg "
                    f"err={math.degrees(err):.1f}deg "
                    f"pos=({self.x:.2f},{self.y:.2f})"
                )
                self.state = 'X_ALIGN'
                self._stuck_last_pos = (self.x, self.y)
                self._stuck_since = self.get_clock().now()
            else:
                ang = max(-1.2, min(1.2, 2.5 * err))
                self._pub(0.0, ang)

        # ── X_ALIGN ───────────────────────────────────────────────────────
        # LANE_DRIVE에서 이미 slot 중심 x까지 접근했으므로 여기서는 검증만 한다.
        # 후진하면서 x를 맞추려고 회전하면 yaw가 틀어져 slot 진입 각도가 깨진다.
        elif self.state == 'X_ALIGN':
            sx = self.target_slot['center_x']
            x_err = sx - self.x
            yaw_err = angle_diff(self.slot_target_yaw, self.yaw)

            if abs(yaw_err) > 0.08:
                ang = max(-0.8, min(0.8, 2.0 * yaw_err))
                self._pub(0.0, ang)
                return

            if abs(x_err) < self.X_ALIGN_TOL:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f"X_ALIGN 완료 — x_err={x_err:.3f}m "
                    f"pos=({self.x:.2f},{self.y:.2f}) → PARK_REVERSE"
                )
                self.state = 'PARK_REVERSE'
                self._stuck_last_pos = (self.x, self.y)
                self._stuck_since = self.get_clock().now()
            else:
                self._pub(0.0, 0.0)
                self.get_logger().warn(
                    f"X_ALIGN 오차 큼 — x_err={x_err:.3f}m, LANE_DRIVE 재시도"
                )
                self.state = 'LANE_DRIVE'
                self._stuck_last_pos = (self.x, self.y)
                self._stuck_since = self.get_clock().now()

        # ── PARK_REVERSE ──────────────────────────────────────────────────
        # 후방 주차: 로봇이 slot 반대 방향을 바라보며 후진(lin < 0).
        # 완료 조건: 후륜이 방지턱 y 위치에 도달해야 한다.
        # LiDAR는 벽/방지턱을 일찍 볼 수 있어 접촉 y 근처에서만 보조로 쓴다.
        elif self.state == 'PARK_REVERSE':
            row = self.target_slot['id'][0]
            park_y = self.PARK_Y_A if row == 'A' else self.PARK_Y_B
            sx = self.target_slot['center_x']

            stopper_enabled = self._stopper_check_enabled()
            stopper_now = stopper_enabled and self._stopper_reached()
            side_enabled = self._side_check_enabled()
            sides_now   = (not side_enabled) or self._sides_clear()
            rear_emrg   = self._rear_emergency()
            self.get_logger().debug(
                f"PARK_REV: pos=({self.x:.2f},{self.y:.2f}) yaw={math.degrees(self.yaw):.1f}deg "
                f"stopper={stopper_now} stopper_enabled={stopper_enabled} "
                f"sides_ok={sides_now} side_enabled={side_enabled} rear_emrg={rear_emrg}"
            )

            y_remaining = abs(park_y - self.y)
            rear_dist  = self._lidar_range(math.pi)
            left_dist  = self._lidar_range(math.pi / 2)
            right_dist = self._lidar_range(-math.pi / 2)

            self.get_logger().debug(
                f"PARK_REV: y_rem={y_remaining:.2f}m "
                f"lidar REAR={rear_dist:.2f} L={left_dist:.2f} R={right_dist:.2f} "
                f"x_err={self.x - sx:.3f}m in_slot={self._in_slot_boundary()}"
            )

            contact_y_ok = self._stopper_contact_y_reached()
            contact_lidar_ok = stopper_now and self._near_stopper_contact_zone()
            in_roi = self._in_slot_boundary()

            if in_roi and (contact_y_ok or contact_lidar_ok):
                stop_reason = 'LiDAR 방지턱 접촉' if contact_lidar_ok else '방지턱 접촉 y 도달'
                self._complete_parking(stop_reason)
                return
            elif contact_y_ok and not in_roi:
                self._pub(0.0, 0.0)
                self.get_logger().warn(
                    f"방지턱 접촉 y에 도달했으나 ROI 밖 — pos=({self.x:.2f},{self.y:.2f}) "
                    f"x_err={self.x - sx:.3f}m — 정지"
                )
                self._highlight_slot(self.target_slot, color='blue')
                self.state = 'DONE'
                return

            # 비상 후방 장애물 감지 (방지턱이 아닌 예상치 못한 장애물)
            if self._rear_emergency():
                self._pub(0.0, 0.0)
                self.get_logger().warn(
                    f"비상: 후방 장애물 {rear_dist:.2f}m — 정지"
                )
                self.state = 'DONE'
                return

            # 측면 충돌 방지 (인접 주차 차량)
            if side_enabled and not sides_now:
                self._pub(0.0, 0.0)
                self.get_logger().warn(
                    f"측면 장애물 — L={left_dist:.2f}m R={right_dist:.2f}m — slot 포기"
                )
                self.slots[self.target_slot['id']]['occupied'] = True
                self._replan()
                return

            # 후진 중에는 yaw를 slot 방향에 고정해 방지턱에 수직으로 접근한다.
            # x 오차는 planar_move의 lateral 속도로 작게 보정해 yaw를 비틀지 않는다.
            x_err = sx - self.x
            yaw_err = angle_diff(self.slot_target_yaw, self.yaw)
            lat_sign = 1.0 if row == 'A' else -1.0
            lat_correction = max(-0.12, min(0.12, lat_sign * 0.5 * x_err))
            ang_correction = max(-0.25, min(0.25, 2.0 * yaw_err))
            speed = min(self.PARK_SPEED, max(0.08, 0.8 * y_remaining))
            if abs(yaw_err) >= 0.12:
                speed *= 0.5
            self._pub(-speed, ang_correction, lat_correction)

        # ── DONE ──────────────────────────────────────────────────────────
        elif self.state == 'DONE':
            self._pub(0.0, 0.0)

    # ── 유틸리티 ──────────────────────────────────────────────────────────

    def _set_target(self, slot):
        self.target_slot = slot
        # 후방 주차: slot 반대 방향을 바라보고 후진 진입.
        # Row A (북쪽, y+): 남향(-π/2) → 후진하면 북쪽으로 진입
        # Row B (남쪽, y-): 북향(+π/2) → 후진하면 남쪽으로 진입
        self.slot_target_yaw = (
            -math.pi / 2 if slot['id'][0] == 'A' else math.pi / 2
        )
        self.get_logger().info(
            f"목표 slot: {slot['id']} "
            f"(x={slot['center_x']}, {'북쪽 후진' if slot['id'][0]=='A' else '남쪽 후진'})"
        )
        self._highlight_slot(slot, color='green')

    def _replan(self):
        """측면 충돌 포기 후 로컬 YAML 기반으로 대체 slot 선택."""
        candidates = [
            s for s in self.slots.values()
            if not s['occupied'] and s['type'] == 'general'
        ]
        candidates.sort(key=lambda s: abs(s['center_x'] + 10.5))
        if candidates:
            self._set_target(candidates[0])
            self.state = 'LANE_DRIVE'
        else:
            self.get_logger().error('빈 slot 없음 — DONE')
            self.state = 'DONE'


def main(args=None):
    rclpy.init(args=args)
    node = ParkingNode()
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
