#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from navigation.navigation_manager import NavigationManager
from vision.vision_manager import VisionManager
from lift.lift_manager import LiftManager
import time
from enum import Enum
import yaml
from ament_index_python.packages import get_package_share_directory

NUMBER_OF_OBJECTS = 3  # Number of objects to be placed in trucks
# Mock modules for testing purposes
mock_modules = [
]

# states for state machine
class PuzzlebotState(Enum):
    INITIALIZING = 0
    IDENTIFY_TRUCKS = 1
    EXPLORING = 2
    PICK = 3
    PLACE = 4
    END = 98
    ERROR = 99

class TruckType(Enum):
    RED = 0
    YELLOW = 1
    GRAY = 2

class PuzzlebotManager(Node):
    def __init__(self):
        super().__init__('puzzlebot_manager')
        self._default_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        
        
        self.get_logger().info("Initializing PuzzlebotManager...")
        self.get_logger().info(f"Mock modules: {', '.join(mock_modules)}")
        
        self.initialize_modules()
        
        self.get_logger().info("Module initialization complete.")
        self.get_logger().info("PuzzlebotManager initialized successfully.")
        
        # Truck locations (poses) are stored in a yaml received as a parameter
        package_path = get_package_share_directory('puzzlebot_manager')
        truck_locations_file = f"{package_path}/config/truck_locations.yaml"
        truck_locations_filepath = self.declare_parameter('truck_locations_file',
                                                        truck_locations_file).value
        exploration_goals_file = f"{package_path}/config/exploration_goals.yaml"
        exploration_goals_filepath = self.declare_parameter('exploration_goals_file',
                                                        exploration_goals_file).value
        self.navigation_manager.load_exploration_goals(exploration_goals_filepath)
        self.navigation_manager.load_truck_locations(truck_locations_filepath)
        
        self.current_state = PuzzlebotState.INITIALIZING
        self.target_truck_type = None # Default truck type
        self.objects_placed = 0  # Counter for placed objects
        
        # 20hz
        self.state_machine_timer = self.create_timer(0.05, self.state_machine_callback, 
                                                     callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup())
        
        self.get_logger().info("PuzzlebotManager node started successfully.")
        
    def initialize_modules(self):
        self.navigation_manager = NavigationManager(self, mock="navigation" in mock_modules)
        self.vision_manager = VisionManager(self, mock="vision" in mock_modules)
        self.lift_manager = LiftManager(self, mock="lift" in mock_modules)
        
        
    def state_machine_callback(self):
        print("state machine callback")
        if self.current_state == PuzzlebotState.INITIALIZING:
            self.get_logger().info("PuzzlebotManager is initializing...")
            self.current_state = PuzzlebotState.IDENTIFY_TRUCKS
            
        elif self.current_state == PuzzlebotState.IDENTIFY_TRUCKS:
            self.get_logger().info("Identifying trucks...")
            
            for i, truck_location in enumerate(self.navigation_manager.truck_locations):
                self.navigation_manager.go_to_truck_location(truck_index=i)
                print("waiting for truck inference")
                time.sleep(5)
                truck_label = self.vision_manager.truck_classify(wait=True)
                self.navigation_manager.truck_named_locations[truck_label] = truck_location
                self.get_logger().info(f"Truck {truck_label} identified at location {truck_location.pose.position.x}, {truck_location.pose.position.y}.")
                
            self.get_logger().info("Trucks identified.")
            self.get_logger().info("Starting exploration...")
            self.current_state = PuzzlebotState.EXPLORING
            while True:
                time.sleep(0.1)
        
        elif self.current_state == PuzzlebotState.EXPLORING:
            
            self.navigation_manager.explore()
            
            if len(self.vision_manager.get_qr_codes()) != 0:
                self.navigation_manager.stop_exploration()
                self.current_state = PuzzlebotState.PICK
        
        elif self.current_state == PuzzlebotState.PICK:
            self.get_logger().info("Picking an object...")
            # Here you would implement the logic to pick an object
            # For now, we will just simulate it
            time.sleep(2)
            
            self.get_logger().info("Object picked.")
            
            self.current_state = PuzzlebotState.PLACE
        
        elif self.current_state == PuzzlebotState.PLACE:
            self.get_logger().info("Placing the object in a truck...")
            # Here you would implement the logic to place an object in a truck
            # For now, we will just simulate it
            time.sleep(2)
            
            self.objects_placed += 1
            self.get_logger().info(f"Object placed. Total objects placed: {self.objects_placed}")
            
            if self.objects_placed >= NUMBER_OF_OBJECTS:
                self.current_state = PuzzlebotState.END
            else:
                self.current_state = PuzzlebotState.EXPLORING
                
        elif self.current_state == PuzzlebotState.END:
            self.get_logger().info("All objects placed. Ending the process.")
            self.current_state = PuzzlebotState.ERROR
            
        elif self.current_state == PuzzlebotState.ERROR:
            self.get_logger().error("An error occurred in the PuzzlebotManager state machine.")
            self.current_state = PuzzlebotState.INITIALIZING
            
def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(8)
    puzzlebot_manager = PuzzlebotManager()
    executor.add_node(puzzlebot_manager)
    try:
        executor.spin()
    except KeyboardInterrupt:
        puzzlebot_manager.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        puzzlebot_manager.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
            