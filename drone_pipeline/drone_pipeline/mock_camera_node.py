import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class MockCameraNode(Node):
    def __init__(self):
        super().__init__('mock_camera_node')
        self.publisher_ = self.create_publisher(Image, '/scout/image_raw', 10)
        self.bridge = CvBridge()
        timer_period = 1.0 / 15.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frame_count = 0
        self.get_logger().info('Mock camera node started, publishing fake frames on /scout/image_raw')

    def timer_callback(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:] = (40, 40, 40)
        cv2.putText(frame, f'MOCK FRAME {self.frame_count}', (150, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.frame_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MockCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
