#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import numpy as np
from copy import deepcopy
import math


class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.global_map_received = False
        self.global_map = None

        self.local_map_subscription = self.create_subscription(
            OccupancyGrid,
            'local_map',
            self.local_map_callback,
            10
        )

        self.global_map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.global_map_callback,
            10
        )

        self.merged_map_publisher = self.create_publisher(
            OccupancyGrid,
            'merged_map',
            10
        )

        self.get_logger().info('Map merger initialized.')

    def global_map_callback(self, msg: OccupancyGrid):
        if not self.global_map_received:
            self.global_map = msg
            self.global_map_received = True
            self.get_logger().info('Global map received and stored.')

    def local_map_callback(self, local_map: OccupancyGrid):
        if not self.global_map_received:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', local_map.header.frame_id, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return

        merged_data = deepcopy(np.array(self.global_map.data, dtype=np.int8).reshape(
            (self.global_map.info.height, self.global_map.info.width)
        ))

        for y in range(local_map.info.height):
            for x in range(local_map.info.width):
                idx = y * local_map.info.width + x
                value = local_map.data[idx]

                if value != 100:
                    continue  # Only care about obstacles

                # Compute world coordinate of local cell
                local_x = x * local_map.info.resolution + local_map.info.origin.position.x
                local_y = y * local_map.info.resolution + local_map.info.origin.position.y

                # Transform to map frame
                ps = PoseStamped()
                ps.header.frame_id = local_map.header.frame_id
                ps.pose.position.x = local_x
                ps.pose.position.y = local_y
                ps.pose.orientation.w = 1.0

                try:
                    transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)
                    map_x = int((transformed.pose.position.x - self.global_map.info.origin.position.x)
                                / self.global_map.info.resolution)
                    map_y = int((transformed.pose.position.y - self.global_map.info.origin.position.y)
                                / self.global_map.info.resolution)

                    if 0 <= map_x < self.global_map.info.width and 0 <= map_y < self.global_map.info.height:
                        merged_data[map_y, map_x] = 100
                except Exception as e:
                    self.get_logger().warn(f"Pose transform failed: {e}")
                    continue

        merged_map = OccupancyGrid()
        merged_map.header.stamp = self.get_clock().now().to_msg()
        merged_map.header.frame_id = 'map'
        merged_map.info = self.global_map.info
        merged_map.data = merged_data.flatten().tolist()
        self.merged_map_publisher.publish(merged_map)


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
