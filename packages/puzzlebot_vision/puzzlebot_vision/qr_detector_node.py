#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from puzzlebot_interfaces.msg import QRCodeArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from puzzlebot_vision.qr_detector.QRDetector import QRDetector

class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')

        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/image')
        self.declare_parameter('compressed_camera_topic', '/camera/image/compressed')

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.compressed_camera_topic = self.get_parameter('compressed_camera_topic').get_parameter_value().string_value

        # Initialize QRDetector and CvBridge
        self.qr_detector = QRDetector()
        self.bridge = CvBridge()

        # Publishers
        self.qr_detections_pub = self.create_publisher(QRCodeArray, '/vision/qr_detections', 10)
        self.qr_image_pub = self.create_publisher(Image, '/vision/qr_detections/image', 10)

        # Subscriber
        if self.compressed_camera_topic:
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.compressed_camera_topic,
                self.compressed_image_callback,
                10
            )
            self.get_logger().info(f'Subscribed to compressed camera topic: {self.compressed_camera_topic}')
        else:
            self.image_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.image_callback,
                10
            )
            self.get_logger().info(f'Subscribed to camera topic: {self.camera_topic}')

    def compressed_image_callback(self, msg):
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process compressed image: {e}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def process_frame(self, frame):
        # Detect QR codes
        detected_qrs = self.qr_detector.detect(frame)

        # Publish QR detections
        qr_array_msg = QRCodeArray()
        qr_array_msg.qrcodes = detected_qrs
        self.qr_detections_pub.publish(qr_array_msg)

        # Draw QR codes on the image
        for qr in detected_qrs:
            x1, y1 = int(qr.x1 * frame.shape[1]), int(qr.y1 * frame.shape[0])
            x2, y2 = int(qr.x2 * frame.shape[1]), int(qr.y2 * frame.shape[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, qr.content, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish the image with QR codes drawn
        qr_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.qr_image_pub.publish(qr_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()