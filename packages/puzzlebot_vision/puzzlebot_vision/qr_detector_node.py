#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped, Pose
from scipy.spatial.transform import Rotation as R
from puzzlebot_interfaces.msg import QRCodeArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import LookupException, ExtrapolationException
from puzzlebot_vision.qr_detector.QRDetector import QRDetector

class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')

        # Declare parameters
        self.declare_parameter('camera_topic', '/video_source/raw')
        self.declare_parameter('compressed_camera_topic', '/video_source/compressed')

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.compressed_camera_topic = self.get_parameter('compressed_camera_topic').get_parameter_value().string_value

        # Initialize QRDetector and CvBridge
        self.qr_detector = QRDetector()
        self.bridge = CvBridge()

        # tf2 buffer and listener
        # Subscriber
        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, qos=qos)

        # Publishers
        self.qr_detections_pub = self.create_publisher(QRCodeArray, '/vision/qr_detections', 10)
        self.qr_image_pub = self.create_publisher(Image, '/vision/qr_detections/image', 10)
        self.qr_image_compressed_pub = self.create_publisher(CompressedImage, '/vision/qr_detections/image/compressed', 10)
        self.qr_pose_pub = self.create_publisher(PoseStamped, '/vision/qr_pose', 10)

        
        if self.compressed_camera_topic:
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.compressed_camera_topic,
                self.compressed_image_callback,
                qos
            )
            self.get_logger().info(f'Subscribed to compressed camera topic: {self.compressed_camera_topic}')
        else:
            self.image_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.image_callback,
                qos
            )
            self.get_logger().info(f'Subscribed to camera topic: {self.camera_topic}')

    def compressed_image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process compressed image: {e}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def process_frame(self, frame):
        detected_qrs = self.qr_detector.detect(frame)

        for qr in detected_qrs:
            # Draw bounding box and content
            x1, y1 = int(qr.x1 * frame.shape[1]), int(qr.y1 * frame.shape[0])
            x2, y2 = int(qr.x2 * frame.shape[1]), int(qr.y2 * frame.shape[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, qr.content, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Build pose in camera_link frame
            pose_cam = Pose()
            pose_cam.position.x = qr.tvec[0]
            pose_cam.position.y = qr.tvec[1]
            pose_cam.position.z = qr.tvec[2]

            r_matrix = cv2.Rodrigues(np.array(qr.rvec))[0]
            q = R.from_matrix(r_matrix).as_quat()
            pose_cam.orientation.x = q[0]
            pose_cam.orientation.y = q[1]
            pose_cam.orientation.z = q[2]
            pose_cam.orientation.w = q[3]

            # Try transforming to base_link frame
            pose_stamped_base = PoseStamped()
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map",  # target
                    "camera_base_link",  # source
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=1.0)
                )
                pose_stamped_base.header.frame_id = "map"
                pose_stamped_base.header.stamp = self.get_clock().now().to_msg()
                pose_stamped_base.pose = tf2_geometry_msgs.do_transform_pose(pose_cam, transform)
            except Exception as e:
                self.get_logger().warn(f'Could not transform QR pose: {e}')
            qr.pose_stamped = pose_stamped_base
            self.qr_pose_pub.publish(pose_stamped_base)

        # Publish annotated image
        qr_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.qr_image_pub.publish(qr_image_msg)

        # Publish QR detections
        qr_array_msg = QRCodeArray()
        qr_array_msg.qrcodes = detected_qrs
        self.qr_detections_pub.publish(qr_array_msg)

        msg_comp = CompressedImage()
        result, endcoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        msg_comp.format = 'jpeg'
        msg_comp.data = endcoded.tobytes()
        self.qr_image_compressed_pub.publish(msg_comp)
        print("published")    

def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
