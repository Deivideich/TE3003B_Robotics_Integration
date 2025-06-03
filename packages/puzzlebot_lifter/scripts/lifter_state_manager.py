#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from puzzlebot_interfaces.srv import LifterMovement
from puzzlebot_lifter.scripts.lifter_loop_controller import LifterControl
from puzzlebot_lifter.lifter_state_class import LifterState

class LifterStateManager(Node):
    def __init__(self):
        super().__init__('lifter_state_manager')
        self.lifter = LifterControl()
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.srv = self.create_service(
            LifterMovement,
            'lifter_move',
            self.handle_lifter_request
        )

    def handle_lifter_request(self, request, response):
        if request.state == LifterState.MOVE_FORK_TO_TOP.value:
            self.lifter.set_state(self.lifter.ARRIBA)
        elif request.state == LifterState.MOVE_FORK_TO_BOTTOM.value:
            self.lifter.set_state(self.lifter.ABAJO)
        else:
            self.lifter.set_state(self.lifter.DETENIDO)
        response.success = True
        return response

    def control_loop(self):
        """This function is called at a regular interval to update the lifter state."""
        self.lifter.update()
        if self.lifter.completed:
            self.get_logger().info("Lifter movement completed.")
            self.lifter.completed = False
        else:
            self.get_logger().info("Lifter is moving...")
        self.get_logger().info(f"Current state: {self.lifter.current_state}")
        self.get_logger().info(f"Overrun start time: {self.lifter.overrun_start_time}")
        self.get_logger().info(f"Current status: {self.lifter.current_status}")

    def destroy_node(self):
        self.lifter.stop()
        self.lifter.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    lifter_state_manager = LifterStateManager()
    rclpy.spin(lifter_state_manager)
    lifter_state_manager.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()