#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import asyncio
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from puzzlebot_interfaces.action import NavigateToPoint as PoseController

class PoseControllerServer(Node):

    def __init__(self):
        super().__init__('pose_controller_server')

        self._action_server = ActionServer(
            self,
            PoseController,
            'navigate_to_point',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Controller parameters
        self.k_lin = 0.5
        self.k_ang = 1.0
        self.goal_tolerance = 0.01
        self.ang_tolerance = 0.05

        # Action state
        self.goal_handle = None
        self.current_goal = None

    def goal_callback(self, goal_request):
        self.get_logger().info('Received new goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.goal_handle = goal_handle
        self.current_goal = goal_handle.request.goal

        feedback_msg = PoseController.Feedback()

        while rclpy.ok() and goal_handle.is_active:
            if self.is_goal_reached():
                self.get_logger().info("Goal reached.")
                self.cmd_pub.publish(Twist())  # stop the robot

                result = PoseController.Result()
                result.success = True

                goal_handle.succeed()
                self.goal_handle = None
                self.current_goal = None
                return result

            # Feedback
            feedback_msg.distance = self.compute_distance_to_goal()
            feedback_msg.angle = self.compute_angular_error()
            feedback_msg.progress = (max(0.0, 1.0 - feedback_msg.distance / 2.0) - 0.5) * 200

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.05)


        self.cmd_pub.publish(Twist())  # stop robot on abort
        goal_handle.abort()
        self.get_logger().warn('Goal aborted.')
        return PoseController.Result(success=False)

    def control_loop(self):
        if self.goal_handle is None or not self.goal_handle.is_active:
            return

        goal = self.current_goal.pose.position
        dx = goal.x - self.x
        dy = goal.y - self.y
        distance = math.hypot(dx, dy)

        cmd = Twist()
        target_theta = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_theta - self.theta)

        if abs(angle_error) > self.ang_tolerance:
            cmd.angular.z = self.k_ang * angle_error
        else:
            cmd.linear.x = self.k_lin * distance

        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def compute_distance_to_goal(self):
        goal = self.current_goal.pose.position
        dx = goal.x - self.x
        dy = goal.y - self.y
        return math.hypot(dx, dy)

    def compute_angular_error(self):
        goal = self.current_goal.pose.position
        dx = goal.x - self.x
        dy = goal.y - self.y
        target_theta = math.atan2(dy, dx)
        return self.normalize_angle(target_theta - self.theta)

    def is_goal_reached(self):
        return self.compute_distance_to_goal() < self.goal_tolerance


def main(args=None):
    rclpy.init(args=args)
    node = PoseControllerServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
