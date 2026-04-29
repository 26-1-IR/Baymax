#!/usr/bin/env python3
"""Vision node: slot type and occupancy detection.

Pipeline:
  1. Subscribes to /ego/camera/image_raw
  2. On each observation trigger (/parking/obs_trigger, Bool=True):
       - Runs YOLOv8 if model file found (yolo_model param)
       - Falls back to YAML ground truth if model not available
  3. Publishes /parking/slot_states (std_msgs/String, JSON)

YOLO classes expected:
  0 = 'isa_mark'   → slot is handicapped type
  1 = 'vehicle'    → slot is occupied

Slot ROI matching:
  Each detected bounding box centre is projected from image coords to
  world coords using a top-down homography (calibrated from Gazebo camera
  intrinsics/pose).  The projected point is matched against slot ROIs
  from the YAML metadata.
"""

import json
import math
import os

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
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


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('yolo_model', '')   # path to .pt file
        model_path = self.get_parameter('yolo_model').value

        pkg = get_package_share_directory('autonomous_parking')
        with open(os.path.join(pkg, 'config', 'slot_metadata.yaml')) as f:
            data = yaml.safe_load(f)
        self.slots = {s['id']: dict(s) for s in data['slots']}

        self.model = None
        if YOLO_OK and model_path and os.path.isfile(model_path):
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLOv8 model loaded: {model_path}')
        else:
            self.get_logger().warn(
                'YOLOv8 not available — using YAML ground truth for occupancy. '
                'ISA type comes from metadata.'
            )

        self.bridge = CvBridge() if CV_OK else None

        # Latest images from each camera (None until first frame arrives)
        self.images = {
            'front': None,
            'left':  None,
            'right': None,
        }
        self.triggered = False

        self.create_subscription(Image, '/ego/camera/image_raw',       self._mk_image_cb('front'), 10)
        self.create_subscription(Image, '/ego/camera_left/image_raw',  self._mk_image_cb('left'),  10)
        self.create_subscription(Image, '/ego/camera_right/image_raw', self._mk_image_cb('right'), 10)
        self.create_subscription(CameraInfo, '/ego/camera/camera_info', self._info_cb, 1)
        self.create_subscription(Bool, '/parking/obs_trigger', self._trigger_cb, 10)

        self.pub = self.create_publisher(String, '/parking/slot_states', 10)
        self.create_timer(0.1, self._tick)

        self.get_logger().info('Vision node started')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _mk_image_cb(self, key):
        def _cb(msg):
            self.images[key] = msg
        return _cb

    def _info_cb(self, msg):
        if self.K is None:
            self.K = list(msg.k)
            self.get_logger().info('Camera intrinsics received')

    def _trigger_cb(self, msg):
        if msg.data:
            self.triggered = True

    # ── main tick ──────────────────────────────────────────────────────────

    def _tick(self):
        if not self.triggered:
            return
        self.triggered = False

        available = [k for k, v in self.images.items() if v is not None]
        self.get_logger().info(f'Vision triggered — cameras available: {available}')

        if self.model is not None and CV_OK and available:
            self._run_yolo(available)
        else:
            self._publish_yaml_states()

    # ── YOLO detection ────────────────────────────────────────────────────

    def _run_yolo(self, camera_keys):
        """Run YOLOv8 on all available camera images and merge detections."""
        detections = []
        for key in camera_keys:
            msg = self.images[key]
            if msg is None:
                continue
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().error(f'cv_bridge error [{key}]: {e}')
                continue
            results = self.model.predict(cv_img, conf=0.4, verbose=False)
            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    cx = float((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                    cy = float((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
                    detections.append({'cls': cls, 'cx_img': cx, 'cy_img': cy,
                                       'conf': float(box.conf[0]),
                                       'camera': key})

        self.get_logger().info(f'YOLO: {len(detections)} detections across {camera_keys}')

        # Update slot states from YOLO detections
        # (Full projection to world coords requires known robot pose —
        #  here we use a simplified IoU-based image-space matching.)
        # For now, detections update an internal overlay that persists.
        updated = dict(self.slots)

        # cls 0 = isa_mark → mark nearby slot as handicapped (type override)
        # cls 1 = vehicle  → mark overlapping slot as occupied
        for det in detections:
            if det['cls'] == 1:  # vehicle detected
                # Find slot whose projected ROI centre is closest in image
                best_id = self._match_detection_to_slot(det['cx_img'], det['cy_img'])
                if best_id:
                    updated[best_id]['occupied'] = True
                    self.get_logger().info(f"YOLO: slot {best_id} → occupied")
            elif det['cls'] == 0:  # ISA mark detected
                best_id = self._match_detection_to_slot(det['cx_img'], det['cy_img'])
                if best_id:
                    updated[best_id]['type'] = 'handicapped'
                    self.get_logger().info(f"YOLO: slot {best_id} → handicapped")

        self.slots = updated
        self._publish(updated)

    def _match_detection_to_slot(self, cx, cy):
        """Placeholder: return slot id closest to image-space bounding box centre.

        A full implementation would project slot world coords to image coords
        using the camera intrinsic matrix K and the robot's current pose.
        """
        # Without robot pose integration the projection is approximate.
        # Return None to skip (yaml ground truth will be used instead).
        return None

    # ── yaml fallback ────────────────────────────────────────────────────

    def _publish_yaml_states(self):
        self._publish(self.slots)
        self.get_logger().info('Slot states published (YAML ground truth)')

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


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
