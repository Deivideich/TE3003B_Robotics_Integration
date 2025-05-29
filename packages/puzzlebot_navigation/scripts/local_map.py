#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import LookupException, ExtrapolationException
import numpy as np
import math


class LocalMapPublisher(Node):
    def __init__(self):
        super().__init__('local_map_publisher')

        # Map parameters
        self.declare_parameter('map_width', 2.0)  # in meters
        self.declare_parameter('map_height', 2.0)
        self.declare_parameter('map_resolution', 0.05)

        # Object drawing parameters
        self.declare_parameter('object_width', 0.2)  # in meters
        self.declare_parameter('object_height', 0.2)

        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.object_width = self.get_parameter('object_width').value
        self.object_height = self.get_parameter('object_height').value

        self.map_width_cells = int(self.map_width / self.map_resolution)
        self.map_height_cells = int(self.map_height / self.map_resolution)

        # Internal storage
        self.latest_qr_pose = None

        # tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for the local map
        self.map_publisher = self.create_publisher(OccupancyGrid, 'local_map', 10)

        # Subscribers
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.qr_pose_subscriber = self.create_subscription(PoseStamped, '/vision/qr_pose', self.qr_pose_callback, 10)

        self.get_logger().info('Local map publisher with QR pose tracking initialized.')

    def qr_pose_callback(self, msg: PoseStamped):
        # Try transforming to base_link frame
        self.latest_qr_pose = PoseStamped()
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",  # target
                "camera_base_link",  # source
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            self.latest_qr_pose.header.frame_id = "map"
            self.latest_qr_pose.header.stamp = self.get_clock().now().to_msg()
            self.latest_qr_pose.pose = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not transform QR pose: {e}')

    def scan_callback(self, msg: LaserScan):
        local_map = -1 * np.ones((self.map_height_cells, self.map_width_cells), dtype=np.int8)

        # Fill in laser scan data
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                map_x = int((x + self.map_width / 2) / self.map_resolution)
                map_y = int((y + self.map_height / 2) / self.map_resolution)

                if 0 <= map_x < self.map_width_cells and 0 <= map_y < self.map_height_cells:
                    local_map[map_y, map_x] = 100
            angle += msg.angle_increment

        # Draw QR object if available
        if self.latest_qr_pose is not None:
            x = self.latest_qr_pose.pose.position.x
            y = self.latest_qr_pose.pose.position.y

            if abs(x) <= self.map_width / 2 and abs(y) <= self.map_height / 2:
                self.draw_object_on_map(local_map, x, y)
            else:
                self.latest_qr_pose = None

        self.publish_map(local_map)

    def draw_object_on_map(self, local_map, x, y):
        half_w = self.object_width / 2
        half_h = self.object_height / 2

        x_min = int(((x - half_w) + self.map_width / 2) / self.map_resolution)
        x_max = int(((x + half_w) + self.map_width / 2) / self.map_resolution)
        y_min = int(((y - half_h) + self.map_height / 2) / self.map_resolution)
        y_max = int(((y + half_h) + self.map_height / 2) / self.map_resolution)

        for map_y in range(y_min, y_max + 1):
            for map_x in range(x_min, x_max + 1):
                if 0 <= map_x < self.map_width_cells and 0 <= map_y < self.map_height_cells:
                    local_map[map_y, map_x] = 100  # same value as obstacle

    def publish_map(self, local_map):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = 'base_link'

        occupancy_grid.info.resolution = self.map_resolution
        occupancy_grid.info.width = self.map_width_cells
        occupancy_grid.info.height = self.map_height_cells
        occupancy_grid.info.origin.position.x = -self.map_width / 2
        occupancy_grid.info.origin.position.y = -self.map_height / 2
        occupancy_grid.info.origin.orientation.w = 1.0

        occupancy_grid.data = local_map.flatten().tolist()
        self.map_publisher.publish(occupancy_grid)


def main(args=None):
    rclpy.init(args=args)
    node = LocalMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
