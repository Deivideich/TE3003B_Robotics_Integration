import rclpy
from rclpy.node import Node
import yaml

from puzzlebot_interfaces.srv import LifterMovement
from geometry_msgs.msg import PoseStamped
from puzzlebot_manager.utils.decorators import mockable
from puzzlebot_lifter.lifter_state_class import LifterState
import time


class LiftManager():
    def __init__(self, node: Node, mock : bool = False):
        
        self.node = node
        self.mock_data = False
        
        self.mock_data = mock
        self.lifter_active = False
        
        self.lifter_client = self.node.create_client(
            LifterMovement,
            '/lifter_movement'
        )
        
        self.node.get_logger().info("Initialized LiftManager...")
    
    def set_lifter_state(self, state: LifterState, wait: bool = True):
        """
        Set the state of the lifter.
        """
        if self.mock_data:
            self.node.get_logger().info(f"Mocking lifter state change to {state.name}")
            time.sleep(5)
            
            
        if not self.lifter_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("Lifter service not available!")
            return False
        
        request = LifterMovement.Request()
        request.state = int(state.value)
        self.lifter_active = True
        future = self.lifter_client.call_async(request)
        future.add_done_callback(self.lifter_response_callback)
        if wait:
            while self.lifter_active:
                time.sleep(0.01)
        return True
        
    def lifter_response_callback(self, future):
        """
        Callback for the lifter response.
        """
        self.lifter_active = False