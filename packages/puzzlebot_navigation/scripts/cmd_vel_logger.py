#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv
import os
from datetime import datetime

class CmdVelOdomLogger(Node):
    def __init__(self):
        super().__init__('cmd_vel_odom_logger')

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Latest message holders
        self.latest_cmd_vel = None
        self.latest_odom = None

        # CSV file setup
        self.filename = 'log.csv'
        self.filepath = os.path.join(os.getcwd(), self.filename)
        self.init_csv()

        # Timer to log data every 100ms
        self.create_timer(0.1, self.log_data)

    def init_csv(self):
        with open(self.filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'timestamp',
                'cmd_vel_linear_x', 'cmd_vel_linear_y', 'cmd_vel_angular_z',
                'odom_pos_x', 'odom_pos_y', 'odom_yaw'
            ])

    def cmd_vel_callback(self, msg: Twist):
        self.latest_cmd_vel = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def log_data(self):
        if self.latest_cmd_vel is None or self.latest_odom is None:
            return

        cmd = self.latest_cmd_vel
        odom = self.latest_odom
        timestamp = datetime.now().isoformat()

        position = odom.pose.pose.position
        orientation_q = odom.pose.pose.orientation
        yaw = self.quaternion_to_yaw(orientation_q)

        row = [
            timestamp,
            cmd.linear.x, cmd.linear.y, cmd.angular.z,
            position.x, position.y, yaw
        ]

        with open(self.filepath, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(row)

    @staticmethod
    def quaternion_to_yaw(q):
        import math
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
