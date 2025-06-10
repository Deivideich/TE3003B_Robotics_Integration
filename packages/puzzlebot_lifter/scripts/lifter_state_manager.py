#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from puzzlebot_interfaces.srv import LifterMovement
from puzzlebot_lifter.lifter_loop_controller import LifterControl
from puzzlebot_lifter.lifter_state_class import LifterState, LifterDirection
import time 

class LifterStateManager(Node):
    def __init__(self):
        super().__init__('lifter_state_manager')
        self.lifter = LifterControl()
        self.srv = self.create_service(
            LifterMovement,
            '/lifter_movement',
            self.handle_lifter_request
        )
        
        self.received_state = LifterState.STOP
        self.completed = False
        self.get_logger().info("Lifter State Manager Node has been started.")

    def handle_lifter_request(self, request, response):
        try:
            self.received_state = LifterState(request.state)
        except ValueError:
            self.get_logger().error(f"Invalid lifter state received: {request.state}")
            response.reached = False
            return response

        self.get_logger().info(f"Received lifter request: {self.received_state.name}")
        
        if self.received_state == LifterState.STOP:
            self.lifter.stop()
            response.reached = self.completed
            
        elif self.received_state == LifterState.MOVE_FORK_TO_BOTTOM:
            self.move_and_wait_for_sensor_trigger(LifterDirection.MOVE_DOWN)
            response.reached = self.completed
            
        elif self.received_state == LifterState.MOVE_FORK_TO_TOP:
            self.move_and_wait_for_sensor_trigger(LifterDirection.MOVE_UP)
            response.reached = self.completed
            
        elif self.received_state == LifterState.MOVE_FORK_TO_MIDDLE:
            self.lifter.timed_move(LifterDirection.MOVE_UP, 1.0)
            self.move_and_wait_for_sensor_trigger(LifterDirection.MOVE_DOWN)
            self.lifter.timed_move(LifterDirection.MOVE_UP, 10.0)
            response.reached = self.completed
            
        elif self.received_state == LifterState.LEAVE_PALLET:
            self.lifter.timed_move(LifterDirection.MOVE_DOWN, 1.0)
            self.lifter.timed_move(LifterDirection.MOVE_UP, 0.5)
            response.reached = self.completed

        return response
    
    # This function moves the lifter in the specified direction and waits for the appropriate sensor to be triggered.
    def move_and_wait_for_sensor_trigger(self, direction, timeout=60.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            sensor_up, sensor_down = self.lifter.read_sensors()
            
            #First check if the bottom sensor is triggered when moving down
            if sensor_down and direction == LifterDirection.MOVE_DOWN:
                self.get_logger().info("Bottom sensor triggered")
                self.lifter.timed_move(direction, 5.0)
                self.get_logger().info("Moving down completed, stopped.")
                self.completed = True
                break
            
            #Then check if the top sensor is triggered when moving up
            elif sensor_up and direction == LifterDirection.MOVE_UP:
                self.get_logger().info("Top sensor triggered")
                self.lifter.timed_move(direction, 10.0)
                self.get_logger().info("Moving up completed, stopped.")
                self.completed = True
                break
            
            else:
                self.lifter.move(direction)
                self.get_logger().info(f"Moving in direction {direction}, waiting for sensor trigger...")
                time.sleep(0.1)
        else:
            self.lifter.stop()
            self.get_logger().warn("Sensor not triggered within timeout.")
            self.completed = False
    
def main(args=None):
    rclpy.init(args=args)
    lifter_state_manager = LifterStateManager()
    rclpy.spin(lifter_state_manager)
    lifter_state_manager.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()