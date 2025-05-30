import rclpy
from rclpy.node import Node
import yaml

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
    
    def load_truck_locations(self, filepath):
        # load yaml
        try:
            with open(filepath, 'r') as file:
                truck_locations = yaml.safe_load(file)
                self.truck_locations = {int(k): PoseStamped(**v) for k, v in truck_locations.items()}
                self.get_logger().info(f"Loaded truck locations: {self.truck_locations}")
        except FileNotFoundError:
            raise FileNotFoundError(f"Truck locations file not found: {filepath}")
        except yaml.YAMLError as e:
            raise ValueError(f"Error parsing truck locations file: {filepath}. Error: {e}")