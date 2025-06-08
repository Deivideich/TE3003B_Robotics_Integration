import rclpy
from rclpy.node import Node
import yaml
import time
from geometry_msgs.msg import PoseStamped
from puzzlebot_manager.utils.decorators import mockable
from puzzlebot_interfaces.msg import QRCodeArray

class VisionManager():
    def __init__(self, node: Node, mock : bool = False):
        
        self.node = node
        self.mock_data = False
        
        self.qr_codes = []
        
        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        self.qr_detections_sub = self.node.create_subscription(
            QRCodeArray,
            '/vision/qr_detections',
            self.qr_detection_callback,
            qos
        )
        
        self.mock_data = mock
        self.node.get_logger().info("Initializing Vision Manager...")
        
    def get_qr_codes(self):
        """
        Get the list of detected QR codes.
        """
        return self.qr_codes
        
    def qr_detection_callback(self, msg: QRCodeArray):
        """
        Callback function to handle received QR code detections.
        """
        self.qr_codes = msg.qrcodes