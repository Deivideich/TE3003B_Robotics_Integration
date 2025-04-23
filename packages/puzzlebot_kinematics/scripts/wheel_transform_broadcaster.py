#!/usr/bin/env python3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import rclpy
from rclpy.node import Node
import time

class FakeWheelPublisher(Node):
    def __init__(self):
        super().__init__('fake_wheel_pub')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.start_time = time.time()
        # self.timer = self.create_timer(0.05, self.publish_state)  # 20 Hz
        self.wheel_vel_update = self.create_subscription(Float64MultiArray, "/wheel_velocities", self.wheel_vel_callback, 10)
        self.joint_position_l = 0
        self.joint_position_r = 0
    
    def wheel_vel_callback(self, msg):
        omega_l = msg.data[0]  # rad/s for left wheel
        omega_r = msg.data[1]  # rad/s for right wheel

        dt = 0.05  # Assuming 20Hz, or compute from time difference for accuracy

        # Accumulate angular position (radians) for each wheel
        self.joint_position_l += omega_l * dt
        self.joint_position_r += omega_r * dt

        # self.get_logger().info(
        #     f"Wheel positions -> Left: {self.joint_position_l:.2f} rad, Right: {self.joint_position_r:.2f} rad"
        # )

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['wheel_l_joint', 'wheel_r_joint']
        joint_state_msg.position = [self.joint_position_l, self.joint_position_r]
        joint_state_msg.velocity = [omega_l, omega_r]

        self.publisher.publish(joint_state_msg)
  
    
    # def publish_state(self):
    #     msg = JointState()
    #     now = self.get_clock().now().to_msg()
    #     msg.header.stamp = now
    #     msg.name = ['wheel_l_joint', 'wheel_r_joint']
    #     elapsed = time.time() - self.start_time
    #     # Example: wheel rotates 1 rad/s
    #     msg.position = [elapsed, -elapsed]
    #     msg.velocity = [1.0, -1.0]
    #     self.publisher.publish(msg)

def main():
    rclpy.init()
    node = FakeWheelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()