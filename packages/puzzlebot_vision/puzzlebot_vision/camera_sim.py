#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Create publishers for both raw and compressed images
        self.image_pub = self.create_publisher(Image, '/camera/image', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)
        
        # Set up camera capture
        self.cap = cv2.VideoCapture(0)  # Use 0 for default webcam
        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            return

        # Create timer for periodic image capture and publishing
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 fps
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        # Convert frame to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(img_msg)

        # Convert frame to CompressedImage message
        compressed_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.compressed_pub.publish(compressed_msg)

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()