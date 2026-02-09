#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesis
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.declare_parameter('camera_topic', '/image_raw')
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)

        # Load YOLO model
        # Replace 'yolov8n.pt' with your trained TurtleBot3 model if needed
        self.model = YOLO('yolov8n.pt')  

        self.get_logger().info(f"YOLO node started, subscribing to {self.camera_topic}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        # Run YOLO detection
        results = self.model(cv_image)[0]  # only first batch

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for r in results.boxes:
            x1, y1, x2, y2 = r.xyxy[0].cpu().numpy()
            score = float(r.conf[0])
            class_id = int(r.cls[0])

            det = Detection2D()
            det.bbox.center.x = (x1 + x2) / 2
            det.bbox.center.y = (y1 + y2) / 2
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            det.results.append(ObjectHypothesis(class_id=class_id, score=score))
            detections_msg.detections.append(det)

        self.pub.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
