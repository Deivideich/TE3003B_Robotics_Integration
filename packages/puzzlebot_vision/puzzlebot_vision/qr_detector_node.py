#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from puzzlebot_interfaces.msg import QRCodeArray
from puzzlebot_interfaces.srv import GetQRsObject
from cv_bridge import CvBridge
import cv2
import numpy as np
from copy import deepcopy
from puzzlebot_vision.qr_detector.QRDetector import QRDetector, QRDetectorTorch
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
import tf2_ros 
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
import time
QR_THRESHOLD = 0.5  # Adjust this threshold based on your needs

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
        self.detected_qrs = []

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

        # Service
        self.get_qrs_service = self.create_service(
            GetQRsObject,
            'get_qrs_objects',
            self.handle_get_qrs_objects
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('QRDetectorNode initialized.')
        
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

    def handle_get_qrs_objects(self, request, response):
        # Check if a frame is available
        if not hasattr(self, 'frame'):
            self.get_logger().warn('No frame available to process.')
            response.qrcodes = []
            return response
        
        response.qrcodes = self.detected_qrs
        self.get_logger().info(f'Detected {len(response.qrcodes)} QR codes.')
        
        return response
    
    def _publish_qr_transform(self, qr):
        transform = TransformStamped()
        transform.header.stamp = self.frame_stamp.to_msg()
        transform.header.frame_id = 'camera_base_link'
        transform.child_frame_id = f'qr_code_{str(qr.content)}'
        transform.transform.translation.x = qr.tvec[0]
        transform.transform.translation.y = qr.tvec[1]
        transform.transform.translation.z = qr.tvec[2]
        transform.transform.rotation.x = qr.rvec[0]
        transform.transform.rotation.y = qr.rvec[1]
        transform.transform.rotation.z = qr.rvec[2]
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def _publish_qr_posestamped(self, qr):
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

        return pose_cam

    def compressed_image_callback(self, msg):
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.frame_stamp = self.get_clock().now()
            self.process_frame(deepcopy(self.frame))
        except Exception as e:
            self.get_logger().error(f'Failed to process compressed image: {e}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_stamp = self.get_clock().now()
            self.process_frame(deepcopy(self.frame))
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def process_frame(self, frame):
        # Detect QR codes
        prev_time = time.time()
        self.detected_qrs = self.qr_detector.detect(frame)
        print(f'Detected qrs in {time.time() - prev_time:.4f} seconds')
        self.valid_qrs = []

        # Draw QR codes on the image
        for qr in self.detected_qrs:
            if np.linalg.norm(qr.tvec) < QR_THRESHOLD:
                # Publish detected qrt transfrom
                self._publish_qr_transform(qr)

                # Build pose in camera_link frame
                self._publish_qr_posestamped(qr)

                # Draw qr in image 
                x1, y1 = int(qr.x1 * frame.shape[1]), int(qr.y1 * frame.shape[0])
                x2, y2 = int(qr.x2 * frame.shape[1]), int(qr.y2 * frame.shape[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, qr.content, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                self.valid_qrs.append(qr)


        # Publish QR array message
        qr_code_array = QRCodeArray()
        qr_code_array.qrcodes = self.valid_qrs
        self.qr_detections_pub.publish(qr_code_array)
            
        # Publish annotated image
        qr_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.qr_image_pub.publish(qr_image_msg)

        # return detected_qrs
        msg_comp = CompressedImage()
        result, endcoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        msg_comp.format = 'jpeg'
        msg_comp.data = endcoded.tobytes()
        self.qr_image_compressed_pub.publish(msg_comp)

def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()