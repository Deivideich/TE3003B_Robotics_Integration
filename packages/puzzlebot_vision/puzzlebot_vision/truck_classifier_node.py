#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from puzzlebot_interfaces.msg import ImageClassification
from puzzlebot_interfaces.srv import ClassifyImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from puzzlebot_vision.truck_classifier.classifier import TruckClassifier
import os

class TruckClassifierNode(Node):
    def __init__(self):
        super().__init__('truck_classifier_node')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_topic', '/video_source/raw')
        self.declare_parameter('compressed_camera_topic', '/video_source/compressed')
        self.declare_parameter('enable_continuous_inference', True)

        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            raise ValueError('Model path parameter is required')
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.compressed_camera_topic = self.get_parameter('compressed_camera_topic').get_parameter_value().string_value
        self.enable_continuous_inference = self.get_parameter('enable_continuous_inference').get_parameter_value().bool_value

        # Validate model path
        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f'Model path not provided or does not exist: {model_path}')
            return

        # Initialize classifier and CvBridge
        try:
            self.classifier = TruckClassifier(model_path)
            self.bridge = CvBridge()
            self.get_logger().info(f'Truck classifier loaded from: {model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load truck classifier: {e}')
            return

        # State for continuous inference
        self.inference_active = True

        # Always create the service
        self.classify_service = self.create_service(
            ClassifyImage,
            '/vision/classify_image',
            self.classify_image_callback
        )

        # Only create continuous inference components if enabled
        if self.enable_continuous_inference:
            self.get_logger().info('Continuous inference mode enabled - creating image subscribers and publishers')
            
            # Publishers
            self.classification_pub = self.create_publisher(
                ImageClassification, 
                '/vision/truck_classification', 
                10
            )
            self.classified_image_pub = self.create_publisher(
                Image, 
                '/vision/truck_classification/image', 
                10
            )

            # Subscribers
            self.active_sub = self.create_subscription(
                Bool,
                '/vision/truck_classify_active',
                self.active_callback,
                10
            )

            # Initialize image subscriber (prefer compressed)
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
        else:
            self.get_logger().info('Continuous inference mode disabled - only service mode available')

        self.get_logger().info('Truck classifier node initialized')

    def active_callback(self, msg):
        """Toggle continuous inference on/off"""
        self.inference_active = msg.data
        status = "activated" if self.inference_active else "deactivated"
        self.get_logger().info(f'Continuous truck classification {status}')

    def compressed_image_callback(self, msg):
        """Process compressed image if continuous inference is active"""
        if not self.inference_active:
            return
            
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process compressed image: {e}')

    def image_callback(self, msg):
        """Process raw image if continuous inference is active"""
        if not self.inference_active:
            return
            
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def process_frame(self, frame):
        """Perform truck classification on a frame"""
        try:
            # Perform inference
            label_id, label_name = self.classifier.inference(frame)
            
            # Create and publish classification message
            classification_msg = ImageClassification()
            classification_msg.label_id = label_id
            classification_msg.label_name = label_name if label_name else "Unknown"
            classification_msg.confidence = 0.0  # Add confidence calculation if needed
            
            self.classification_pub.publish(classification_msg)
            
            # Draw classification result on image
            text = f"ID: {label_id}"
            if label_name:
                text += f" ({label_name})"
            
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Publish classified image
            classified_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.classified_image_pub.publish(classified_image_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to classify frame: {e}')

    def classify_image_callback(self, request, response):
        """Service callback for on-demand image classification"""
        try:
            # Determine which image to use (prefer compressed if both are provided)
            if request.image_comp.data:
                # Use compressed image
                np_arr = np.frombuffer(request.image_comp.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            elif request.image.data:
                # Use raw image
                frame = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')
            else:
                self.get_logger().error('No image data provided in service request')
                response.image_classification.label_id = -1
                response.image_classification.label_name = "Error: No image data"
                response.image_classification.confidence = 0.0
                return response
            
            # Perform inference
            label_id, label_name = self.classifier.inference(frame)
            
            # Populate response
            response.image_classification.label_id = label_id
            response.image_classification.label_name = label_name if label_name else "Unknown"
            response.image_classification.confidence = 0.0  # Add confidence calculation if needed
            
            self.get_logger().info(f'Service classification result: ID={label_id}, Name={label_name}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to classify image in service: {e}')
            response.image_classification.label_id = -1
            response.image_classification.label_name = f"Error: {str(e)}"
            response.image_classification.confidence = 0.0
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TruckClassifierNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()