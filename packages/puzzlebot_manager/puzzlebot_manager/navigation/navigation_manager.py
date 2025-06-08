import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import yaml
import time
from geometry_msgs.msg import PoseStamped
from puzzlebot_manager.utils.decorators import mockable
from puzzlebot_interfaces.action import ControllerAction

class NavigationManager():
    def __init__(self, node: Node, mock : bool = False):
        
        self.node = node
        self.mock_data = False
        
        self.mock_data = mock
        
        self.navigation_action_client = ActionClient(
            self.node, 
            ControllerAction,
            'controller_server',
        )
        
        self.current_exploration_goal = 0
        self.truck_locations = []
        self.exploration_goals = []
        self.exploration_goal_active = False
        
        self.node.get_logger().info("Initializing NavigationManager...")
    
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
    
    def load_exploration_goals(self, filepath):
        """
        Load exploration goals from a YAML file.
        The file should contain a list of poses for exploration.
        """
        self.node.get_logger().info(f"Loading exploration goals from {filepath}")
        try:
            with open(filepath, 'r') as file:
                exploration_goals_data = yaml.safe_load(file)
                self.node.get_logger().info(f"Exploration goals loaded.")
                # transform to PoseStamped
                self.exploration_goals = []
                for goal in exploration_goals_data:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = goal['position'][0]
                    pose.pose.position.y = goal['position'][1]
                    pose.pose.position.z = goal['position'][2]
                    pose.pose.orientation.x = goal['orientation'][0]
                    pose.pose.orientation.y = goal['orientation'][1]
                    pose.pose.orientation.z = goal['orientation'][2]
                    pose.pose.orientation.w = goal['orientation'][3]
                    self.exploration_goals.append(pose)
        except Exception as e:
            self.node.get_logger().error(f"Failed to load exploration goals from {filepath}: {e}")
            self.exploration_goals = []
        return self.exploration_goals
    
    def explore(self):
        if self.mock_data:
            self.node.get_logger().info("Mocking exploration...")
            return True
        
        
        # Start new exploration if not currently exploring
        if not self.exploration_goal_active:
            self.node.get_logger().info("Going for next exploration goal...")
            goal_msg = ControllerAction.Goal()
            goal_msg.goal = self.exploration_goals[self.current_exploration_goal]
            self.current_exploration_goal += 1
            if self.current_exploration_goal >= len(self.exploration_goals):
                self.current_exploration_goal = 0
            goal_msg.ignore_obstacles = True
            
            # Send goal asynchronously
            navigation_goal_future = self.navigation_action_client.send_goal_async(goal_msg)
            # rclpy.spin_until_future_complete(self.node, send_goal_future)
            navigation_goal_future.add_done_callback(self.exploration_goal_callback)
            
            self.exploration_goal_active = True
            self.node.get_logger().info(f"Exploration goal sent")
        
        return False
    
    def exploration_goal_callback(self, future):
        self.navigation_goal_handle = future.result()

        self._get_result_future = self.navigation_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.exploration_result_callback)
        
    def exploration_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.node.get_logger().info(f"Exploration goal {self.current_exploration_goal - 1} completed successfully.")
        else:
            self.node.get_logger().error(f"Exploration goal {self.current_exploration_goal - 1} failed.")
        
        self.exploration_goal_active = False
        
    def stop_exploration(self):
        """
        Manually stop ongoing exploration.
        """
        if self.exploration_goal_active:
            self.node.get_logger().info("Stopping ongoing exploration...")
            # Cancel the current exploration goal
            cancel_future = self.navigation_goal_handle.cancel_goal_async()
            self.exploration_goal_active = False
            self.node.get_logger().info("Exploration stopped.")
        return False