#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class Publisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)

        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.cam_sub = self.create_subscription(Image, '/video_source/raw', self.cam_callback, 10)
        self.comp_pub = self.create_publisher(CompressedImage, '/video_source/compressed', 10)
        self.msg_comp = CompressedImage()

    def cam_callback(self, msg):
        try:
            self.frame = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.width != 0 and self.height != 0:
                self.frame = cv2.resize(self.frame, (self.width, self.height))
            result, endcoded = cv2.imencode('.jpg', self.frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            self.msg_comp.format = 'jpeg'
            self.msg_comp.data = endcoded.tobytes()
            self.comp_pub.publish(self.msg_comp)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return

def main(args=None):
    rclpy.init(args=args)
    
    try:
        carVision_ = Publisher()
        rclpy.spin(carVision_)
        carVision_.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()