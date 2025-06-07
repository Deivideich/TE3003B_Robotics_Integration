#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
import math

from puzzlebot_manager_interfaces.srv import getQRsObject  # Replace with actual service type
from puzzlebot_manager_interfaces.action import GoToPose  # Replace with actual action type

class GoToObjectNode(Node):
    def __init__(self):
        super().__init__('go_to_object')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.qr_service_client = self.create_client(getQRsObject, 'qr_service')
        self.action_client = ActionClient(self, GoToPose, 'go_to_pose')
        self.timer = self.create_timer(1.0, self.main_loop)

    def main_loop(self):
        if not self.qr_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('QR service not available')
            return

        req = getQRsObject.Request()
        future = self.qr_service_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result():
            self.get_logger().warn('No QR data received')
            return

        qr_pose = future.result().pose  # PoseStamped
        qr_content = future.result().content

        # Define 4 relative poses (in QR frame)
        offsets = [
            (0.025, 0.0),
            (-0.025, 0.0),
            (0.0, -0.025),
            (0.0, 0.025)
        ]
        transformed_poses = []
        for dx, dy in offsets:
            pose = PoseStamped()
            pose.header = qr_pose.header
            pose.pose.position.x = qr_pose.pose.position.x + dx
            pose.pose.position.y = qr_pose.pose.position.y + dy
            pose.pose.position.z = qr_pose.pose.position.z
            pose.pose.orientation = qr_pose.pose.orientation
            try:
                trans = self.tf_buffer.lookup_transform(
                    'map', pose.header.frame_id, rclpy.time.Time())
                map_pose = do_transform_pose(pose, trans)
                transformed_poses.append(map_pose)
            except Exception as e:
                self.get_logger().warn(f'TF transform failed: {e}')
                return

        # Get robot's current pose in map frame
        try:
            robot_pose = PoseStamped()
            robot_pose.header.frame_id = 'base_link'
            robot_pose.header.stamp = self.get_clock().now().to_msg()
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            robot_map_pose = do_transform_pose(robot_pose, trans)
        except Exception as e:
            self.get_logger().warn(f'Could not get robot pose: {e}')
            return

        # Find closest transformed pose
        min_dist = float('inf')
        closest_pose = None
        for pose in transformed_poses:
            dx = pose.pose.position.x - robot_map_pose.pose.position.x
            dy = pose.pose.position.y - robot_map_pose.pose.position.y
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                closest_pose = pose

        if closest_pose is None:
            self.get_logger().warn('No valid pose found')
            return

        # Send action goal to controller
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Controller action server not available')
            return

        goal_msg = GoToPose.Goal()
        goal_msg.target_pose = closest_pose
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by controller')
            return
        self.get_logger().info('Goal sent to controller')

def main(args=None):
    rclpy.init(args=args)
    node = GoToObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()