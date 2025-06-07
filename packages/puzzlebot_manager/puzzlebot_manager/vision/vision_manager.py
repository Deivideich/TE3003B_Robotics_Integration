import rclpy
from rclpy.node import Node
import yaml
import time
from geometry_msgs.msg import PoseStamped
from puzzlebot_manager.utils.decorators import mockable

class VisionManager():
    def __init__(self, node: Node, mock : bool = False):
        
        self.node = node
        self.mock_data = False
        
        self.mock_data = mock
        self.node.get_logger().info("Initializing Vision Manager...")
        
    @mockable(return_value=[], delay=0.5, mock=False)
    def detect_qrs(self):
        """
        Mock method to simulate QR code detection.
        Returns a list of detected QR codes.
        """
        self.node.get_logger().info("Detecting QR codes...")
        # Simulate detection delay
        time.sleep(0.1)
        
        # Return mock data
        return [
        ]