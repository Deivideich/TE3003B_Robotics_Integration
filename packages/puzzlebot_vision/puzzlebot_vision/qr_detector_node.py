#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from puzzlebot_interfaces.msg import QRCodeArray
from puzzlebot_interfaces.srv import GetQRsObject
from cv_bridge import CvBridge
import cv2
import numpy as np
from puzzlebot_vision.qr_detector.QRDetector import QRDetector
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')

        # Declare parameters
        self.declare_parameter('camera_topic', '/video_source/raw')
        self.declare_parameter('compressed_camera_topic', '/video_source/compressed')
        self.declare_parameter('time_threshold', 2.0) # Maximum time to wait for a frame before processing
        

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.compressed_camera_topic = self.get_parameter('compressed_camera_topic').get_parameter_value().string_value
        self.time_threshold = self.get_parameter('time_threshold').get_parameter_value().double_value

        # Initialize QRDetector and CvBridge
        self.qr_detector = QRDetector()
        self.bridge = CvBridge()

        # Publishers
        # self.qr_detections_pub = self.create_publisher(QRCodeArray, '/vision/qr_detections', 10)
        self.qr_image_pub = self.create_publisher(Image, '/vision/qr_detections/image', 10)
        # Service
        self.get_qrs_service = self.create_service(
            GetQRsObject,
            'get_qrs_objects',
            self.handle_get_qrs_objects
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('QRDetectorNode initialized.')

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

    def handle_get_qrs_objects(self, request, response):
        # Check if a frame is available
        if not hasattr(self, 'frame'):
            self.get_logger().warn('No frame available to process.')
            response.qrcodes = []
            return response
        
        response.qrcodes = self.process_frame(self.frame)
        self.get_logger().info(f'Detected {len(response.qrcodes)} QR codes.')
        
        return response

    def compressed_image_callback(self, msg):
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.frame_stamp = self.get_clock().now()
            # self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process compressed image: {e}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_stamp = self.get_clock().now()
            # self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def process_frame(self, frame):
        # Detect QR codes
        detected_qrs = self.qr_detector.detect(frame)

        # Publish QR detections
        # qr_array_msg = QRCodeArray()
        # qr_array_msg.qrcodes = detected_qrs
        # self.qr_detections_pub.publish(qr_array_msg)

        # Draw QR codes on the image
        for qr in detected_qrs:
            x1, y1 = int(qr.x1 * frame.shape[1]), int(qr.y1 * frame.shape[0])
            x2, y2 = int(qr.x2 * frame.shape[1]), int(qr.y2 * frame.shape[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, qr.content, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish the image with QR codes drawn
        qr_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.qr_image_pub.publish(qr_image_msg)

        # Broadcast TF for each detected QR code
        for qr in detected_qrs:
            transform = TransformStamped()
            transform.header.stamp = self.frame_stamp.to_msg()
            transform.header.frame_id = 'camera_base_link'
            transform.child_frame_id = f'qr_code_{qr.content}'
            transform.transform.translation.x = qr.tvec[0]
            transform.transform.translation.y = qr.tvec[1]
            transform.transform.translation.z = qr.tvec[2]
            transform.transform.rotation.x = qr.rvec[0]
            transform.transform.rotation.y = qr.rvec[1]
            transform.transform.rotation.z = qr.rvec[2]
            transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(transform)
        # Return detected QR codes
        self.get_logger().info(f'Processed frame with {len(detected_qrs)} QR codes detected.')

        return detected_qrs

def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()