import threading
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class MultiCameraSaver(Node):
    def __init__(self):
        super().__init__('multi_camera_saver')
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.cameras = {
            'center': {
                'topic': '/ego/camera/image_raw',
                'save_dir': Path.home() / 'workspace' / 'Baymax' / 'datasets' / 'parking' / 'images' / 'raw_center',
                'latest_frame': None,
                'saved_count': 0,
            },
            'left': {
                'topic': '/ego/camera_left/image_raw',
                'save_dir': Path.home() / 'workspace' / 'Baymax' / 'datasets' / 'parking' / 'images' / 'raw_left',
                'latest_frame': None,
                'saved_count': 0,
            },
            'right': {
                'topic': '/ego/camera_right/image_raw',
                'save_dir': Path.home() / 'workspace' / 'Baymax' / 'datasets' / 'parking' / 'images' / 'raw_right',
                'latest_frame': None,
                'saved_count': 0,
            },
        }

        for name, cam in self.cameras.items():
            cam['save_dir'].mkdir(parents=True, exist_ok=True)
            existing = sorted(cam['save_dir'].glob('frame_*.jpg'))
            if existing:
                last_num = max(int(p.stem.split('_')[1]) for p in existing)
                cam['saved_count'] = last_num + 1

            self.create_subscription(
                Image,
                cam['topic'],
                lambda msg, cam_name=name: self.image_callback(msg, cam_name),
                10
            )

            self.get_logger().info(f'[{name}] topic: {cam["topic"]}')
            self.get_logger().info(f'[{name}] save dir: {cam["save_dir"]}')

        self.get_logger().info('Press ENTER to save center/left/right together. Type q and ENTER to quit.')

    def image_callback(self, msg: Image, cam_name: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.cameras[cam_name]['latest_frame'] = frame
        except Exception as e:
            self.get_logger().error(f'[{cam_name}] failed to convert image: {e}')

    def save_all_frames(self):
        with self.lock:
            for name, cam in self.cameras.items():
                if cam['latest_frame'] is None:
                    self.get_logger().warning(f'[{name}] no frame received yet')
                    return

            frames = {
                name: cam['latest_frame'].copy()
                for name, cam in self.cameras.items()
            }

        for name, cam in self.cameras.items():
            filename = cam['save_dir'] / f'frame_{cam["saved_count"]:06d}.jpg'
            ok = cv2.imwrite(str(filename), frames[name])
            if ok:
                self.get_logger().info(f'[{name}] saved: {filename.name}')
                cam['saved_count'] += 1
            else:
                self.get_logger().error(f'[{name}] failed to write: {filename}')

    def all_ready(self):
        with self.lock:
            return all(cam['latest_frame'] is not None for cam in self.cameras.values())


def main():
    rclpy.init()
    node = MultiCameraSaver()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            user_input = input()
            if user_input.strip().lower() == 'q':
                break
            node.save_all_frames()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
