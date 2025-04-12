#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros.transform_broadcaster import TransformBroadcaster
import math
from scipy.spatial.transform import Rotation as R

class WheelTFBroadcaster(Node):
    def __init__(self):
        super().__init__('wheel_tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_base = 0.332  # Distance between left and right wheels (in meters)
        self.wheel_radius = 0.065  # Radius of the wheels (in meters)

        # Declare and initialize cmd_vel subscriber
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Wheel position offsets
        self.left_wheel_offset = 0.095
        self.right_wheel_offset = -0.095
        
        # Timer to broadcast transform
        self.timer = self.create_timer(0.05, self.broadcast_transforms)  # 20 Hz

        # Initialize wheel angle variables to simulate continuous rotation
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

    def cmd_vel_callback(self, msg):
        """Callback function to receive cmd_vel messages and extract velocities"""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def broadcast_transforms(self):
        """Broadcast the transform from base_footprint to wheels and simulate wheel rotation"""
        now = self.get_clock().now().to_msg()

        # Compute wheel velocities using inverse differential drive kinematics
        v_l = (self.linear_velocity - self.angular_velocity * self.wheel_base / 2) / self.wheel_radius
        v_r = (self.linear_velocity + self.angular_velocity * self.wheel_base / 2) / self.wheel_radius

        # Update wheel angles based on their velocities
        self.left_wheel_angle += v_l * 0.05  # Increment the angle for left wheel
        self.right_wheel_angle += v_r * 0.05  # Increment the angle for right wheel

        # Simulate wheel rotation based on the calculated velocities
        self.simulate_wheel_rotation(self.left_wheel_angle, self.right_wheel_angle, now)

        # Broadcast transform for base_footprint to odom
        quat = R.from_euler('z', self.theta).as_quat()  # Yaw only -> rotation around Z
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(t)


    def simulate_wheel_rotation(self, left_wheel_angle, right_wheel_angle, now):
        """Simulate wheel rotation based on velocity"""
        for wheel_name, y_offset, angle in [('wheel_l_link', self.left_wheel_offset, left_wheel_angle),
                                            ('wheel_r_link', self.right_wheel_offset, right_wheel_angle)]:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_footprint'
            t.child_frame_id = wheel_name

            t.transform.translation.x = 0.05
            t.transform.translation.y = y_offset
            t.transform.translation.z = 0.05

            # -------- Static orientation: roll = 90 deg, pitch = -90 deg, yaw = 0 --------
            roll = 0.0
            pitch = 0.0
            yaw = 1.57

            cr = math.cos(roll / 2)
            sr = math.sin(roll / 2)
            cp = math.cos(pitch / 2)
            sp = math.sin(pitch / 2)
            cy = math.cos(yaw / 2)
            sy = math.sin(yaw / 2)

            # Quaternion from RPY (roll, pitch, yaw)
            q_static_x = sr * cp * cy - cr * sp * sy
            q_static_y = cr * sp * cy + sr * cp * sy
            q_static_z = cr * cp * sy - sr * sp * cy
            q_static_w = cr * cp * cy + sr * sp * sy

            # -------- Dynamic wheel spin (around X axis) --------
            half_angle = angle / 2.0
            sin_half = math.sin(half_angle)
            cos_half = math.cos(half_angle)

            q_spin_x = sin_half
            q_spin_y = 0.0
            q_spin_z = 0.0
            q_spin_w = cos_half

            # -------- Combine static orientation and dynamic rotation --------
            qx = q_static_w * q_spin_x + q_static_x * q_spin_w + q_static_y * q_spin_z - q_static_z * q_spin_y
            qy = q_static_w * q_spin_y - q_static_x * q_spin_z + q_static_y * q_spin_w + q_static_z * q_spin_x
            qz = q_static_w * q_spin_z + q_static_x * q_spin_y - q_static_y * q_spin_x + q_static_z * q_spin_w
            qw = q_static_w * q_spin_w - q_static_x * q_spin_x - q_static_y * q_spin_y - q_static_z * q_spin_z

            # -------- Set transform --------
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
