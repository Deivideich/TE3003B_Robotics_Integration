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

NUMBER_OF_OBJECTS = 3  # Number of objects to be placed in trucks
IDENTIFY_TRUCK_FIRST = True  # Flag to identify truck types before exploring


mock_modules = [
    'navigation',
    'vision',
]

# states for state machine
class PuzzlebotState(Enum):
    INITIALIZING = 0
    EXPLORING = 1
    PICK = 2
    SELECT_TARGET_TRUCK = 3
    GOING_TO_TARGET_TRUCK = 4
    GOING_TO_UNKNOWN_TRUCK = 5
    IDENTIFYING_TRUCK = 6
    PLACING_OBJECT = 7
    RETURNING_FROM_TRUCK = 8
    COMPLETED = 9
    ERROR = 99

class TruckType(Enum):
    RED = 0
    YELLOW = 1
    GRAY = 2

class PuzzlebotManager(Node):
    def __init__(self):
        super().__init__('puzzlebot_manager')
        self.mock_data = False
        
        self.get_logger().info("Initializing PuzzlebotManager...")
        self.get_logger().info(f"Mock modules: {', '.join(mock_modules)}")
        
        self.initialize_modules()
        
        self.get_logger().info("Module initialization complete.")
        self.get_logger().info("PuzzlebotManager initialized successfully.")
        
        # Truck locations (poses) are stored in a yaml received as a parameter
        package_path = self.get_package_share_directory('puzzlebot_manager')
        truck_locations_file = f"{package_path}/config/truck_locations.yaml"
        truck_locations_filepath = self.declare_parameter('truck_locations_file',
                                                        truck_locations_file).value
        self.navigation_manager.load_truck_locations(truck_locations_filepath)
        
        self.current_state = PuzzlebotState.INITIALIZING
        self.target_truck_type = None # Default truck type
        self.objects_placed = 0  # Counter for placed objects
        
        self.run()
        
    def initialize_modules(self):
        self.navigation_manager = NavigationManager(self, mock=('navigation' in mock_modules))
        self.vision_manager = VisionManager(self, mock=('vision' in mock_modules))
        self.lift_manager = LiftManager(self, mock=('lift' in mock_modules))
    
    def run(self):
        """
        Main loop for the PuzzlebotManager.
        """
        if self.current_state == PuzzlebotState.INITIALIZING:
            self.get_logger().info("Puzzlebot is initializing...")
            
            # if IDENTIFY_TRUCK_FIRST is True, we will first go to every truck and identify its type
            if IDENTIFY_TRUCK_FIRST:
                self.get_logger().info("Identifying truck types before exploring.")
                self.current_state = PuzzlebotState.SELECT_TARGET_TRUCK
            else:
                self.get_logger().info("Skipping truck identification, proceeding to exploration.")
                self.current_state = PuzzlebotState.EXPLORING
        
        elif self.current_state == PuzzlebotState.CHECK_FOR_FOUND_QR_CODES:
            if len(self.navigation_manager.get_found_qr_codes()) > 0:
                self.current_qr_code = self.navigation_manager.get_found_qr_codes()[0]
                self.get_logger().info(f"Found another QR code while going to truck: {self.current_qr_code}.")
                self.navigation_manager.navigate_to_qr_code(self.current_qr_code)
            else:
                self.get_logger().info("No more QR codes found, initializing exploration.")
                self.current_state = PuzzlebotState.EXPLORING
        
        elif self.current_state == PuzzlebotState.EXPLORING:
            self.navigation_manager.explore()
            detected_qr_codes = self.vision_manager.detect_qr_codes()
            if len(detected_qr_codes) > 0:
                self.current_state = PuzzlebotState.PICK
                self.get_logger().info("Detected QR codes, transitioning to PICK state.")
                self.current_qr_code = self.vision_manager.closest_qr_code(detected_qr_codes)
                
        elif self.current_state == PuzzlebotState.PICK:
            self.get_logger().info(f"Navigating to QR code: {self.current_qr_code}")
            self.navigation_manager.navigate_to_qr_code(self.current_qr_code)
            self.lift_manager.pick_object(self.current_qr_code)
            self.target_truck_type = self.vision_manager.get_truck_type(self.current_qr_code)
            self.current_state = PuzzlebotState.GOING_TO_TRUCK
            
        elif self.current_state == PuzzlebotState.SELECT_TARGET_TRUCK:
            
            if self.target_truck_type is None:
                self.get_logger().error("No target truck type, only identifying trucks.")
                # check if we have identified all trucks
                if len(self.navigation_manager.known_truck_locations) >= len(TruckType):
                    self.get_logger().info("All truck types identified, proceeding to pick objects.")
                    self.current_state = PuzzlebotState.EXPLORING
            
            if self.target_truck_type in self.navigation_manager.known_truck_locations:
                self.truck_location = self.navigation_manager.known_truck_locations[self.target_truck_type]
                self.get_logger().info(f"Truck type {self.target_truck_type.name} already identified.")
                self.current_state = PuzzlebotState.GOING_TO_TARGET_TRUCK
                
            else:
                self.get_logger().info(f"Truck type {self.target_truck_type.name} not yet found, navigating to closest unknown truck.")
                self.truck_location = self.navigation_manager.get_closest_unknown_truck()
                self.get_logger().info(f"Closest unknown truck location: {self.truck_location}")
                self.current_state = PuzzlebotState.GOING_TO_UNKNOWN_TRUCK
                
        elif self.current_state == PuzzlebotState.GOING_TO_UNKNOWN_TRUCK:
            self.get_logger().info(f"Navigating to unknown truck at {self.truck_location}.")
            self.navigation_manager.navigate_to_pose(self.truck_location)
            self.current_state = PuzzlebotState.IDENTIFYING_TRUCK
            
        elif self.current_state == PuzzlebotState.IDENTIFYING_TRUCK:
            self.get_logger().info("Identifying truck...")
            truck_type = self.vision_manager.identify_truck()
            if truck_type is None:
                self.get_logger().error("Failed to identify truck, retrying...")
                time.sleep(2)
                self.current_state = PuzzlebotState.IDENTIFYING_TRUCK
            
            self.navigation_manager.add_known_truck_location(self.target_truck_type, self.truck_location)
            
            if truck_type == self.target_truck_type:
                self.get_logger().info(f"Identified truck type: {truck_type.name}.")
                self.current_state = PuzzlebotState.PLACING_OBJECT
            else:
                self.get_logger().info(f"Identified truck type {truck_type.name}, which does not match target {self.target_truck_type.name}.")
                self.current_state = PuzzlebotState.SELECT_TARGET_TRUCK
                
        elif self.current_state == PuzzlebotState.GOING_TO_TARGET_TRUCK:
            self.get_logger().info(f"Navigating to target truck of type {self.target_truck_type.name}.")
            self.navigation_manager.navigate_to_pose(self.truck_location)
            self.current_state = PuzzlebotState.PLACING_OBJECT
            
        elif self.current_state == PuzzlebotState.PLACING_OBJECT:
            self.get_logger().info("Placing object in truck...")
            self.navigation_manager.approach_truck(self.target_truck_type)
            self.lift_manager.place_object(self.target_truck_type)
            self.get_logger().info(f"Object placed in {self.target_truck_type.name} truck.")
            self.current_state = PuzzlebotState.RETURNING_FROM_TRUCK
            
        elif self.current_state == PuzzlebotState.RETURNING_FROM_TRUCK:
            self.get_logger().info("Returning from truck...")
            self.navigation_manager.go_back(3)
            
            if self.objects_placed >= NUMBER_OF_OBJECTS:
                self.get_logger().info("All objects placed, completing task.")
                self.current_state = PuzzlebotState.COMPLETED
            else:
                self.objects_placed += 1
                self.get_logger().info(f"Objects placed: {self.objects_placed}/{NUMBER_OF_OBJECTS}.")
                self.current_state = PuzzlebotState.CHECK_FOR_FOUND_QR_CODES
                
        
        elif self.current_state == PuzzlebotState.COMPLETED:
            self.get_logger().info("Puzzlebot task completed successfully.")
            while rclpy.ok():
                rclpy.spin_once(self)
                
        elif self.current_state == PuzzlebotState.ERROR:
            self.get_logger().error("An error occurred in the PuzzlebotManager. Shutting down.")
            while rclpy.ok():
                rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    puzzlebot_manager = PuzzlebotManager()
    
    puzzlebot_manager.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()