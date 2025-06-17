#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from puzzlebot_interfaces.srv import ClassifyImage
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import os

class TruckClassifyTest(Node):
    def __init__(self):
        super().__init__('truck_classify_test')
        self.bridge = CvBridge()
        
        # Declare parameter
        self.declare_parameter('image_file', '')
        
        # Get parameter
        self.image_file = self.get_parameter('image_file').get_parameter_value().string_value
        
        if not self.image_file:
            self.get_logger().error('Image file parameter not provided. Use --ros-args -p image_file:="/path/to/image.jpg"')
            return
        
        # Create service client
        self.classify_client = self.create_client(ClassifyImage, '/vision/classify_image')
        
        # Wait for service to be available
        while not self.classify_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for truck classification service...')
        
        self.get_logger().info('Truck classification service is available')
        
        # Send classification request immediately
        self.send_and_process_request()

    def send_and_process_request(self):
        """Send image classification request and process response"""
        future = self.send_classification_request(self.image_file)
        
        if future is None:
            self.get_logger().error('Failed to send classification request')
            return
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            classification = response.image_classification
            
            self.get_logger().info(
                f'Classification result: '
                f'ID={classification.label_id}, '
                f'Name="{classification.label_name}", '
                f'Confidence={classification.confidence:.2f}'
            )
            
            print(f"Label ID: {classification.label_id}")
            print(f"Label Name: {classification.label_name}")
            print(f"Confidence: {classification.confidence:.2f}")
        else:
            self.get_logger().error('Service call failed')

    def send_classification_request(self, image_path):
        """Send image classification request"""
        if not os.path.exists(image_path):
            self.get_logger().error(f'Image file not found: {image_path}')
            return None

        try:
            # Load image
            frame = cv2.imread(image_path)
            
            if frame is None:
                self.get_logger().error(f'Failed to load image: {image_path}')
                return None
            cv2.imshow('Image', frame)
            cv2.waitKey(0)
            # Convert to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

            # Create compressed image (optional - service prefers compressed)
            _, encoded_img = cv2.imencode('.jpg', frame)
            compressed_msg = CompressedImage()
            compressed_msg.format = 'jpeg'
            compressed_msg.data = encoded_img.tobytes()

            # Create service request
            request = ClassifyImage.Request()
            request.image = image_msg
            request.image_comp = compressed_msg

            # Call service
            self.get_logger().info(f'Sending classification request for: {image_path}')
            future = self.classify_client.call_async(request)
            
            return future

        except Exception as e:
            self.get_logger().error(f'Failed to create classification request: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    
    # Create test node
    test_node = TruckClassifyTest()
    
    try:
        # Node will handle the request in its constructor
        pass
            
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')
    except Exception as e:
        test_node.get_logger().error(f'Test failed: {e}')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()