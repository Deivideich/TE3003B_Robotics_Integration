#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from navigation.navigation_manager import NavigationManager
from vision.vision_manager import VisionManager
from lift.lift_manager import LiftManager
from puzzlebot_lifter.lifter_state_class import LifterState

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
    UP = 0
    DOWN = 1

class PuzzlebotManager(Node):
    def __init__(self):
        super().__init__('puzzlebot_manager')
        self._default_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        
        
        self.get_logger().info("Initializing PuzzlebotManager...")
        self.get_logger().info(f"Mock modules: {', '.join(mock_modules)}")
        
        self.initialize_modules()
        
        self.get_logger().info("Module initialization complete.")
        self.get_logger().info("PuzzlebotManager initialized successfully.")
        
        # 20hz
        self.current_state = PuzzlebotState.UP
        self.state_machine_timer = self.create_timer(0.05, self.state_machine_callback, 
                                                     callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup())
        
        self.get_logger().info("PuzzlebotManager node started successfully.")
        
    def initialize_modules(self):
        self.lift_manager = LiftManager(self, mock="lift" in mock_modules)
        
        
    def state_machine_callback(self):
        print("state machine callback")
        if self.current_state == PuzzlebotState.UP:
            print("Going up")
            self.lift_manager.set_lifter_state(LifterState.MOVE_FORK_TO_TOP, wait=True)
            print("Done going up")
            self.current_state = PuzzlebotState.DOWN
            time.sleep(5)
            
        elif self.current_state == PuzzlebotState.DOWN:
            print("Going down")
            self.lift_manager.set_lifter_state(LifterState.MOVE_FORK_TO_BOTTOM, wait=True)
            print("Done")
            while True:
                time.sleep(1)

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
