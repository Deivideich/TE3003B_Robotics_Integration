#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from navigation.navigation_manager import NavigationManager
from vision.vision_manager import VisionManager

from enum import Enum

mock_modules = [
    'navigation',
    'vision',
]

# states for state machine
class PuzzlebotState(Enum):
    INITIALIZING = 0
    EXPLORING = 1
    PICK = 2
    GOING_TO_TRUCK = 3
    IDENTIFYING_TRUCK = 4
    PLACING_OBJECT = 5
    RETURNING_FROM_TRUCK = 6
    COMPLETED = 7
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
        
        self.current_state = PuzzlebotState.INITIALIZING
        
        self.run()
        
    def initialize_modules(self):
        self.navigation_manager = NavigationManager(self, mock=('navigation' in mock_modules))
        self.vision_manager = VisionManager(self, mock=('vision' in mock_modules))
        
    def run(self):
        """
        Main loop for the PuzzlebotManager.
        """
        if self.current_state == PuzzlebotState.INITIALIZING:
            self.get_logger().info("Puzzlebot is initializing...")
            # Simulate some initialization logic
            self.current_state = PuzzlebotState.EXPLORING
            
        elif self.current_state == PuzzlebotState.EXPLORING:
            self.navigation_manager.explore()
            detected_qr_codes = self.vision_manager.detect_qr_codes()
            if len(detected_qr_codes) > 0:
                self.current_state = PuzzlebotState.PICK
                self.get_logger().info("Detected QR codes, transitioning to PICK state.")
                self.current_qr_code = self.vision_manager.closest_qr_code(detected_qr_codes)
                
                
        elif self.current_state == PuzzlebotState.PICK:
            

def main(args=None):
    rclpy.init(args=args)
    puzzlebot_manager = PuzzlebotManager()
    
    puzzlebot_manager.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()