#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class PhotoTaker(Node):
    def __init__(self):
        super().__init__('photo_taker')
        
        # Parameter for frequency (photos per second)
        self.declare_parameter('frequency', 2.0)
        self.frequency = self.get_parameter('frequency').value
        
        # Get folder name from user input
        self.folder_name = input("Enter folder name for saving images: ")
        
        # Create folder if it doesn't exist
        os.makedirs(self.folder_name, exist_ok=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscription to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',  # Adjust topic name as needed
            self.image_callback,
            10)
        
        # Create timer for taking photos
        period = 1.0 / self.frequency
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.latest_image = None
        self.get_logger().info(f'PhotoTaker node started. Saving to folder: {self.folder_name}')

    def image_callback(self, msg):
        self.latest_image = msg

    def timer_callback(self):
        if self.latest_image is not None:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
                
                # Generate filename with datetime
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = f"{self.folder_name}/{timestamp}.jpg"
                
                # Save image
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'Saved image: {filename}')
                
            except Exception as e:
                self.get_logger().error(f'Error saving image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    photo_taker = PhotoTaker()
    
    try:
        rclpy.spin(photo_taker)
    except KeyboardInterrupt:
        photo_taker.get_logger().info('Node stopped cleanly')
    except Exception as e:
        photo_taker.get_logger().error(f'Error: {str(e)}')
    finally:
        photo_taker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()