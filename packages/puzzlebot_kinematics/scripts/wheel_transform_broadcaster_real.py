#!/usr/bin/env python3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import rclpy
from rclpy.node import Node
import time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Float32

class RealWheelPublisher(Node):
    def __init__(self):
        super().__init__('real_wheel_pub')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.start_time = time.time()
        
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        # Angular velocity subscribers
        self.wL_subscriber = self.create_subscription(Float32, 'VelocityEncL', self.wL_callback, qos_profile=qos_profile_sub) # wL topic subscriber
        self.wR_subscriber = self.create_subscription(Float32, 'VelocityEncR', self.wR_callback, qos_profile=qos_profile_sub) # wR topic subscriber
        self.joint_position_l = 0
        self.joint_position_r = 0
        self.wL = 0.0
        self.wR = 0.0
        self.timer = self.create_timer(0.05, self.publish_state)
        self.dt = 0.05  # Assuming 20Hz, or compute from time difference for accuracy
    
    def publish_state(self):
        # Calculate the elapsed time since the start
        # Accumulate angular position (radians) for each wheel
        self.joint_position_l += self.wL * self.dt
        self.joint_position_r += self.wR * self.dt
        # Create a JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['wheel_l_joint', 'wheel_r_joint']
        joint_state_msg.position = [self.joint_position_l, self.joint_position_r]
        joint_state_msg.velocity = [self.wL, self.wR]
        # Publish the message
        self.publisher.publish(joint_state_msg)
        # Log the wheel positions
    
 # Update left motor angular velocity
    def wL_callback(self, msg):
        self.wL = msg.data
        #self.get_logger().info('OmegaL: {}'.format(msg.data))
    
    # Update right motor angular velocity
    def wR_callback(self, msg):
        self.wR = msg.data
        #Sself.get_logger().info('OmegaR: {}'.format(msg.data))

def main():
    rclpy.init()
    node = RealWheelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()