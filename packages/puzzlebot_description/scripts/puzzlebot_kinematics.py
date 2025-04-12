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
        self.wheel_base = 0.332
        self.wheel_radius = 0.065

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.left_wheel_offset = 0.095
        self.right_wheel_offset = -0.095
        self.timer = self.create_timer(0.05, self.broadcast_transforms)

        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def broadcast_transforms(self):
        now = self.get_clock().now().to_msg()

        # ------- Integrate robot pose using dead reckoning -------
        dt = 0.05
        self.theta += self.angular_velocity * dt
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt

        v_l = (self.linear_velocity - self.angular_velocity * self.wheel_base / 2) / self.wheel_radius
        v_r = (self.linear_velocity + self.angular_velocity * self.wheel_base / 2) / self.wheel_radius

        self.left_wheel_angle += v_l * dt
        self.right_wheel_angle += v_r * dt

        self.simulate_wheel_rotation(self.left_wheel_angle, self.right_wheel_angle, now)

        quat = R.from_euler('z', self.theta).as_quat()
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
        for wheel_name, y_offset, angle in [('wheel_l_link', self.left_wheel_offset, left_wheel_angle),
                                            ('wheel_r_link', self.right_wheel_offset, right_wheel_angle)]:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base_footprint'
            t.child_frame_id = wheel_name

            t.transform.translation.x = 0.05
            t.transform.translation.y = y_offset
            t.transform.translation.z = 0.05

            roll = 0.0
            pitch = 0.0
            yaw = 1.57

            cr = math.cos(roll / 2)
            sr = math.sin(roll / 2)
            cp = math.cos(pitch / 2)
            sp = math.sin(pitch / 2)
            cy = math.cos(yaw / 2)
            sy = math.sin(yaw / 2)

            q_static_x = sr * cp * cy - cr * sp * sy
            q_static_y = cr * sp * cy + sr * cp * sy
            q_static_z = cr * cp * sy - sr * sp * cy
            q_static_w = cr * cp * cy + sr * sp * sy

            half_angle = angle / 2.0
            sin_half = math.sin(half_angle)
            cos_half = math.cos(half_angle)

            q_spin_x = sin_half
            q_spin_y = 0.0
            q_spin_z = 0.0
            q_spin_w = cos_half

            qx = q_static_w * q_spin_x + q_static_x * q_spin_w + q_static_y * q_spin_z - q_static_z * q_spin_y
            qy = q_static_w * q_spin_y - q_static_x * q_spin_z + q_static_y * q_spin_w + q_static_z * q_spin_x
            qz = q_static_w * q_spin_z + q_static_x * q_spin_y - q_static_y * q_spin_x + q_static_z * q_spin_w
            qw = q_static_w * q_spin_w - q_static_x * q_spin_x - q_static_y * q_spin_y - q_static_z * q_spin_z

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
