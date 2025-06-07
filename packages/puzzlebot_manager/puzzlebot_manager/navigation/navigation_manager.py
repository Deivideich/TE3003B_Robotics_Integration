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
        self.node.get_logger().info(f"Loading truck locations from {filepath}")
        try:
            with open(filepath, 'r') as file:
                truck_locations_data = yaml.safe_load(file)
                self.node.get_logger().info(f"Truck locations loaded.")
                # transform to PoseStamped
                self.truck_locations = []
                for truck in truck_locations_data:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = truck['position'][0]
                    pose.pose.position.y = truck['position'][1]
                    pose.pose.position.z = truck['position'][2]
                    pose.pose.orientation.x = truck['orientation'][0]
                    pose.pose.orientation.y = truck['orientation'][1]
                    pose.pose.orientation.z = truck['orientation'][2]
                    pose.pose.orientation.w = truck['orientation'][3]
                    self.truck_locations.append(pose)
        except Exception as e:
            self.node.get_logger().error(f"Failed to load truck locations from {filepath}: {e}")
            self.truck_locations = []
        return self.truck_locations
                