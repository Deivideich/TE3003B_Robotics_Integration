#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateWheelTracker(Node):

    def __init__(self):
        super().__init__('joint_state_wheel_tracker')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self.left_wheel_joint_name = 'wheel_l_joint'
        self.right_wheel_joint_name = 'wheel_r_joint'

        self.initialized = False
        self.initial_left_pos = 0.0
        self.initial_right_pos = 0.0

    def joint_state_callback(self, msg):
        try:
            # Find indices of the wheel joints
            left_index = msg.name.index(self.left_wheel_joint_name)
            right_index = msg.name.index(self.right_wheel_joint_name)

            left_pos = msg.position[left_index]
            right_pos = msg.position[right_index]

            if not self.initialized:
                self.initial_left_pos = left_pos
                self.initial_right_pos = right_pos
                self.initialized = True
                self.get_logger().info("Initial wheel positions recorded.")

            # Calculate displacement since start
            left_displacement = left_pos - self.initial_left_pos
            right_displacement = right_pos - self.initial_right_pos

            self.get_logger().info(
                f"Left wheel: {left_displacement:.3f} rad | Right wheel: {right_displacement:.3f} rad"
            )

        except ValueError as e:
            self.get_logger().warn(f"Wheel joint names not found in joint_states: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateWheelTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
