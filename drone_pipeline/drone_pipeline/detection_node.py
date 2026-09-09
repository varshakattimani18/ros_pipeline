import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from drone_msgs.msg import RawDetection
import os
import cv2

MODEL_PATH = '/home/varsha/drone_ws/models/best.pt'
CONFIDENCE_THRESHOLD = 0.5

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.declare_parameter('drone_id', 'flamingo')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value

        self.save_dir = f'/home/varsha/drone_ws/detections/{self.drone_id}'
        os.makedirs(self.save_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.get_logger().info(f'Loading YOLO model from {MODEL_PATH}')
        self.model = YOLO(MODEL_PATH)

        image_topic = f'/{self.drone_id}/image_raw'
        detections_topic = f'/{self.drone_id}/raw_detections'

        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.publisher_ = self.create_publisher(RawDetection, detections_topic, 10)
        self.detection_counter = 0
        self.get_logger().info(
            f'Detection node started for "{self.drone_id}", subscribed to {image_topic}'
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        results = self.model(frame, verbose=False)[0]

        for box in results.boxes:
            confidence = float(box.conf[0])
            if confidence < CONFIDENCE_THRESHOLD:
                continue
            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2

            det_id = self.detection_counter
            image_path = os.path.join(self.save_dir, f'detection_{det_id}.jpg')
            cv2.imwrite(image_path, frame)

            det_msg = RawDetection()
            det_msg.header = msg.header
            det_msg.detection_id = det_id
            det_msg.pixel_x = center_x
            det_msg.pixel_y = center_y
            det_msg.confidence = confidence
            det_msg.class_name = class_name
            det_msg.image_width = width
            det_msg.image_height = height
            self.publisher_.publish(det_msg)

            self.get_logger().info(
                f'[{self.drone_id}] Published detection #{det_id}: {class_name} '
                f'at ({center_x:.0f}, {center_y:.0f}) confidence={confidence:.2f}'
            )
            self.detection_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
