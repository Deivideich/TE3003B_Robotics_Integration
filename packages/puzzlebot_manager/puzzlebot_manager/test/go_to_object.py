#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from rclpy.action import ActionClient
import math
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from puzzlebot_interfaces.srv import GetQRsObject
from puzzlebot_interfaces.action import ControllerAction
import tf_transformations

DEBUG = True
class GoToObjectNode(Node):
    def __init__(self):
        super().__init__('go_to_object')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.qr_service_client = self.create_client(GetQRsObject, 'get_qrs_objects')
        self.action_client = ActionClient(self, ControllerAction, 'controller_server', callback_group=self.callback_group)

        if DEBUG:
            self.pose_pub = self.create_publisher(PoseStamped, 'debug/target_pose', 10)

        self.timer = self.create_timer(0.5, self.main_loop, callback_group=self.timer_callback_group)

    def main_loop(self):
        if not self.qr_service_client.service_is_ready():
            self.get_logger().warn('QR service not available')
            return

        req = GetQRsObject.Request()
        future = self.qr_service_client.call_async(req)
        future.add_done_callback(self.qr_callback)

        # Disable timer to prevent repeated calls
        self.timer.cancel()

    def qr_callback(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn(f'Failed to call QR service: {e}')
            return
        
        if len(result.qrcodes) == 0:
            self.get_logger().info('No QR codes detected')
            return

        qrs_results = result

        print (f'Detected QR codes: {[qr.content for qr in qrs_results.qrcodes]}')

        try:
            tf = self.tf_buffer.lookup_transform('map', f'qr_code_{qrs_results.qrcodes[0].content}', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return

        x_offset = 0.0 
        y_offset = -0.025

        self.get_logger().info(f'Obtained Map to QR code {qrs_results.qrcodes[0].content} transform')

        
        pose = PoseStamped()
        pose.header.frame_id = f'qr_code_{qrs_results.qrcodes[0].content}'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x_offset
        pose.pose.position.y = 0.0
        pose.pose.position.z = y_offset
        
        # Calculate orientation to look at the QR code (object)
        # Since the x axis is pointing upward, we need to rotate around the z axis
        angle = math.atan2(-y_offset, -x_offset) + math.pi  # +180 degrees to face the QR
        angle = (angle + 2 * math.pi) % (2 * math.pi)  # Normalize to [0, 2pi)

        # Face opposite to the z-axis (i.e., rotate -90 degrees around Y axis)
        q = tf_transformations.quaternion_from_euler(0, math.pi/2, 0)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        try:
            closest_pose = do_transform_pose(pose.pose, tf)
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return


        if closest_pose is None:
            self.get_logger().warn('No valid transformed pose found')
            return

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Controller action server not available')
            return


        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = closest_pose

        if DEBUG:
            self.get_logger().info(f'Sending goal pose: {goal_pose}')
            self.pose_pub.publish(goal_pose)
            

        goal_msg = ControllerAction.Goal()
        goal_msg.goal = goal_pose
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by controller')
            return
        self.get_logger().info('Goal accepted and sent to controller')


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
