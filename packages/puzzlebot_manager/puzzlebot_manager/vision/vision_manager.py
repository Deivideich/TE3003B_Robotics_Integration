import rclpy
from rclpy.node import Node
import yaml

from geometry_msgs.msg import PoseStamped
from puzzlebot_manager.utils.decorators import mockable

class VisionManager():
    def __init__(self, node: Node, mock : bool = False):
        
        self.node = node
        self.mock_data = False
        
        self.mock_data = mock
        self.node.get_logger().info("Initializing Vision Manager...")