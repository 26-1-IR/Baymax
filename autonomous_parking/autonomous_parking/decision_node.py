#!/usr/bin/env python3
"""의사 결정 노드: 사용자 자격 기반 슬롯 선택.

구독 토픽:
  /parking/slot_states  (std_msgs/String, vision_node에서 보낸 JSON)

발행 토픽:
  /parking/target_slot  (std_msgs/String, 슬롯 ID 예: "A3")
  /parking/no_slot      (std_msgs/Bool,   유효한 슬롯 없음)

파라미터:
  user_credential  (string)  'general' | 'handicapped'   기본값: 'general'

의사 결정 파이프라인:
  1. /parking/slot_states에서 슬롯 로드
  2. 점유된 슬롯 제외
  3. 사용자 자격으로 필터링:
       general     → 장애인 전용 슬롯 제외
       handicapped → 모든 빈 슬롯 허용 (장애인 전용 우선)
  4. 진입점(-10.5, 0)에서의 거리 기준 정렬
  5. 최적 슬롯 ID 발행
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class DecisionNode(Node):
    LANE_ENTRY_X = -10.5

    def __init__(self):
        super().__init__('decision_node')

        self.declare_parameter('user_credential', 'general')
        self.credential = self.get_parameter('user_credential').value.lower()
        self.get_logger().info(f'사용자 자격: {self.credential}')

        self.slot_states = {}
        self.published = False    # 관측 사이클당 한 번만 publish

        self.create_subscription(String, '/parking/slot_states', self._states_cb, 10)

        self.pub_target = self.create_publisher(String, '/parking/target_slot', 10)
        self.pub_no_slot = self.create_publisher(Bool, '/parking/no_slot', 10)

        self.get_logger().info('Decision node 준비 — slot states 대기 중')

    # ── 콜백 함수 ─────────────────────────────────────────────────────────

    def _states_cb(self, msg):
        if self.published:
            self.get_logger().debug('이미 slot 선택 완료 — 중복 slot_states 무시')
            return
        try:
            slots = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON 파싱 오류: {e}')
            return

        self.slot_states = {s['id']: s for s in slots}
        self.get_logger().info(
            f'{len(self.slot_states)}개 slot states 수신 — slot 선택 시작'
        )
        self._select_and_publish()

    # ── 의사 결정 로직 ─────────────────────────────────────────────────────

    def _select_and_publish(self):
        # 1단계: 빈 슬롯만 선택
        candidates = [
            s for s in self.slot_states.values() if not s['occupied']
        ]

        # 2단계: 사용자 자격별 필터링
        if self.credential == 'general':
            candidates = [s for s in candidates if s['type'] == 'general']
        # 장애인 사용자: 모든 빈 슬롯 허용

        if not candidates:
            self.get_logger().warn(
                f'자격 "{self.credential}"에 해당하는 유효한 슬롯 없음'
            )
            msg = Bool()
            msg.data = True
            self.pub_no_slot.publish(msg)
            return

        # 3단계: 정렬 (장애인 사용자는 장애인 슬롯 우선, 그다음 거리순)
        def sort_key(s):
            type_priority = 0 if (
                self.credential == 'handicapped' and s['type'] == 'handicapped'
            ) else 1
            dist = abs(s['center_x'] - self.LANE_ENTRY_X)
            return (type_priority, dist)

        candidates.sort(key=sort_key)
        best = candidates[0]

        self.get_logger().info(
            f"slot 선택: {best['id']} "
            f"(type={best['type']}, x={best['center_x']}, y={best['center_y']})"
        )

        msg = String()
        msg.data = best['id']
        self.pub_target.publish(msg)
        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
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
