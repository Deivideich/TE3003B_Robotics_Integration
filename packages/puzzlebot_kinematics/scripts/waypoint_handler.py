#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from puzzlebot_interfaces.action import NavigateToPoint  # <-- Replace with actual package name


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')
        self._action_client = ActionClient(self, NavigateToPoint, 'navigate_to_point')

        self.waypoints = self.load_waypoints()
        self.current_index = 0

        self.timer = self.create_timer(1.0, self.try_send_goal)

    def load_waypoints(self):
        def make_pose(x, y):
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0  # Facing forward
            return pose

        return [
            make_pose(1.0, 0.0),
            make_pose(1.0, 1.0),
            make_pose(0.0, 1.0),
            make_pose(0.0, 0.0)
        ]

    def try_send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server...')
            return

        if self.current_index >= len(self.waypoints):
            self.get_logger().info('All waypoints visited.')
            self.timer.cancel()
            return

        goal_msg = NavigateToPoint.Goal()
        goal_msg.goal = self.waypoints[self.current_index]

        self.get_logger().info(f"Sending goal #{self.current_index + 1}")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.timer.cancel()  # Pause timer while executing

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected!')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Distance: {feedback.distance:.2f} m, "
            f"Angle Error: {feedback.angle:.2f} rad"
        )

    def result_callback(self, future):
        result_wrap = future.result()
        result = result_wrap.result

        if result_wrap.status != 4:  # SUCCEEDED
            self.get_logger().warn(f"Goal failed with status {result_wrap.status}")
            return

        if result.success:
            self.get_logger().info(f"Goal #{self.current_index + 1} succeeded.")
            self.current_index += 1
            self.timer.reset()
        else:
            self.get_logger().warn(f"Goal #{self.current_index + 1} reported failure.")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
