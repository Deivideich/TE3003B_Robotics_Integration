import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from puzzlebot_manager.utils.decorators import mockable

class NavigationManager():
    def __init__(self, node: Node, mock : bool = False):
        
        self.node = node
        self.mock_data = False
        
        self.mock_data = mock
        self.node.get_logger().info("Initializing NavigationManager...")
        
    @mockable(return_value="Mocked publish goal pose", delay=1, mock=False)
    def publish_goal_pose(self, pose : PoseStamped):
        """
        Mocked method to simulate going to a pose.
        """
        self.node.get_logger().info(f"Going to pose: {pose}")
        return True