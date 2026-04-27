import threading
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ManualImageSaver(Node):
    def __init__(self):
        super().__init__('manual_image_saver')
        self.bridge = CvBridge()

        self.declare_parameter(
            'topic',
            '/ego/camera/image_raw'
        )
        self.declare_parameter(
            'save_dir',
            str(Path.home() / 'workspace' / 'Baymax' / 'datasets' / 'parking' / 'images' / 'raw')
        )

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.save_dir = Path(self.get_parameter('save_dir').get_parameter_value().string_value)
        self.save_dir.mkdir(parents=True, exist_ok=True)

        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.saved_count = 0

        existing = sorted(self.save_dir.glob('frame_*.jpg'))
        if existing:
            last_num = max(int(p.stem.split('_')[1]) for p in existing)
            self.saved_count = last_num + 1

        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'Subscribed to: {self.topic}')
        self.get_logger().info(f'Save dir: {self.save_dir}')
        self.get_logger().info('Press ENTER to save the latest frame. Type q and ENTER to quit.')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def save_latest_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                self.get_logger().warning('No frame received yet.')
                return
            frame = self.latest_frame.copy()

        filename = self.save_dir / f'frame_{self.saved_count:06d}.jpg'
        ok = cv2.imwrite(str(filename), frame)
        if ok:
            self.get_logger().info(f'Saved: {filename.name}')
            self.saved_count += 1
        else:
            self.get_logger().error(f'Failed to write: {filename}')


def main():
    rclpy.init()
    node = ManualImageSaver()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            user_input = input()
            if user_input.strip().lower() == 'q':
                break
            node.save_latest_frame()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()