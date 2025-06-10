import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import yaml
import time
from geometry_msgs.msg import PoseStamped,Twist
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
        
        self.navigation_goal_debug_pub = self.node.create_publisher(
            PoseStamped,
            '/navigation/goal_debug',
            10
        )

        self.cmd_publisher = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.current_exploration_goal = 0
        self.truck_locations = []
        self.truck_named_locations = {
            'red_truck': None,
            'yellow_truck': None,
            'gray_truck': None
        }
        self.exploration_goals = []
        self.navigation_goal_active = False
        
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
    
    def go_to_truck_location(self, truck_index: int = -1, truck_type : str = "", wait = False):
        """
        Send a navigation goal to the controller server to go to a truck location.
        If wait is True, wait for the result.
        """
        if self.mock_data:
            self.node.get_logger().info("Mocking navigation to truck location...")
            return True
        
        goal = None
        if truck_index >= 0 and truck_index < len(self.truck_locations):
            goal = self.truck_locations[truck_index]
            self.node.get_logger().info(f"Sending navigation goal to truck {truck_index}...")
        
        elif truck_type in self.truck_named_locations:
            goal = self.truck_locations[self.truck_named_locations[truck_type]]
            self.node.get_logger().info(f"Sending navigation goal to truck type '{truck_type}'...")
        
        if goal is None:
            self.node.get_logger().error(f"Invalid truck index or type: {truck_index}, {truck_type}")
            return False
        # Send goal asynchronous
        self.send_navigation_goal(goal, wait)
        
        return True
    
    def send_navigation_goal(self, goal: PoseStamped, wait = False, ignore_obstacles = False, no_plan = False):
        """
        Send a navigation goal to the controller server.
        If wait is True, wait for the result.
        """
        if self.mock_data:
            self.node.get_logger().info("Mocking navigation goal...")
            return True
        
        # Start new navigation goal if not currently navigating
        if not self.navigation_goal_active:
            self.node.get_logger().info("Sending navigation goal...")
            goal_msg = ControllerAction.Goal()
            goal_msg.goal = goal
            goal_msg.ignore_obstacles = ignore_obstacles
            goal_msg.no_plan = no_plan
            
            # Send goal asynchronous
            self.navigation_goal_active = True
            self.node.get_logger().info("Waiting for server...")
            self.navigation_action_client.wait_for_server()
            self.navigation_goal_debug_pub.publish(goal_msg.goal)  # Publish debug goal
            navigation_goal_future = self.navigation_action_client.send_goal_async(goal_msg)
            navigation_goal_future.add_done_callback(self.navigation_goal_callback)
            
            
            self.node.get_logger().info(f"Navigation goal sent")
            
            if wait:
                while self.navigation_goal_active:
                    time.sleep(0.1)  # Wait until the goal is completed
    
    def cmd_navigation(self, is_forward: bool = True, speed: float = 0.05, duration: float = 3.0, wait: bool = False):
        """
        Command the robot to move forward or backward for a certain duration.
        """
        if self.mock_data:
            self.node.get_logger().info("Mocking cmd_navigation...")
            return True

        twist = Twist()
        twist.linear.x = speed if is_forward else -speed
        twist.angular.z = 0.0

        self.node.get_logger().info(
            f"Publishing cmd_vel: linear.x={twist.linear.x}, duration={duration}s"
        )

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_publisher.publish(twist)
            if not wait:
                break
            time.sleep(0.1)

        # Stop the robot after moving
        stop_twist = Twist()
        self.cmd_publisher.publish(stop_twist)
        self.node.get_logger().info("cmd_navigation completed.")

        return True


    def explore(self):
        if self.mock_data:
            self.node.get_logger().info("Mocking exploration...")
            return True
        
        # Start new exploration if not currently exploring
        if not self.navigation_goal_active:
            self.node.get_logger().info("Going for next exploration goal...")
            goal_msg = ControllerAction.Goal()
            goal_msg.goal = self.exploration_goals[self.current_exploration_goal]
            self.current_exploration_goal += 1
            if self.current_exploration_goal >= len(self.exploration_goals):
                self.current_exploration_goal = 0
            goal_msg.ignore_obstacles = True
            
            # Send goal asynchronous
            self.navigation_action_client.wait_for_server()
            self.navigation_goal_debug_pub.publish(goal_msg.goal)  # Publish debug goal
            navigation_goal_future = self.navigation_action_client.send_goal_async(goal_msg)
            navigation_goal_future.add_done_callback(self.navigation_goal_callback)
            
            self.navigation_goal_active = True
            self.node.get_logger().info(f"Exploration goal sent")
        
        return False
    
    def navigation_goal_callback(self, future):
        self.navigation_goal_handle = future.result()

        self._get_result_future = self.navigation_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigation_result_callback)
        
    def navigation_result_callback(self, future):
        result = future.result().result
        
        self.navigation_goal_active = False
        
    def stop_exploration(self):
        """
        Manually stop ongoing exploration.
        """
        if self.navigation_goal_active:
            self.node.get_logger().info("Stopping ongoing exploration...")
            # Cancel the current exploration goal
            cancel_future = self.navigation_goal_handle.cancel_goal_async()
            self.cancel_complete = False
            cancel_future.add_done_callback(self.cancel_exploration_callback)
            while not self.cancel_complete:
                time.sleep(0.01)
            self.navigation_goal_active = False
            self.node.get_logger().info("Exploration stopped.")
        return False
    
    def cancel_exploration_callback(self, future):
        self.cancel_complete = True
        
    def publish_goal_debug(self, goal: PoseStamped):
        """
        Publish a debug goal for visualization.
        """
        if self.mock_data:
            self.node.get_logger().info("Mocking goal debug publishing...")
            return
        
        self.node.get_logger().info(f"Publishing debug goal: {goal}")
        self.navigation_goal_debug_pub.publish(goal)