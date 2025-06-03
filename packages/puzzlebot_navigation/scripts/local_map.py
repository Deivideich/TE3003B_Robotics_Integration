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
        self.declare_parameter('map_width', 1.0)  # in meters
        self.declare_parameter('map_height', 1.0)
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
        self.local_map = -1 * np.ones((self.map_height_cells, self.map_width_cells), dtype=np.int8)
        self.latest_qr_pose = None
        self.laser_scan_merged = None

        # tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for the local map
        self.map_publisher = self.create_publisher(OccupancyGrid, 'local_map', 10)
        self.local_map_scan_publisher = self.create_publisher(LaserScan, 'local_map_merged_scan', 10)

        # Subscribers
        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.qr_pose_subscriber = self.create_subscription(PoseStamped, '/vision/qr_pose', self.qr_pose_callback, qos)
        
        self.local_map_timer = self.create_timer(0.1, self.publish_map)  # Adjust timer as needed

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
        # TODO: CHANGE THIS FOR REAL ROBOT msg.angle_min and max + pi
        local_map = -1 * np.ones((self.map_height_cells, self.map_width_cells), dtype=np.int8)
        # Fill in laser scan data
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                map_x = int((x + self.map_width / 2) / self.map_resolution)
                map_y = int((y + self.map_height / 2) / self.map_resolution)
                if (0 <= map_x < self.map_width_cells) and (0 <= map_y < self.map_height_cells):
                    local_map[map_y, map_x] = 100
            angle += msg.angle_increment

        # Draw QR object if available
        modified_ranges = msg.ranges
        if self.latest_qr_pose is not None:
            x = self.latest_qr_pose.pose.position.x
            y = self.latest_qr_pose.pose.position.y

            if abs(x) <= self.map_width / 2 and abs(y) <= self.map_height / 2:
                self.draw_object_on_map(local_map, x, y)
                modified_ranges = self.draw_object_on_laser_scan(msg, local_map, x, y)
            else:
                self.latest_qr_pose = None
        
        self.local_map = local_map
        self.laser_scan_merged = msg
        self.laser_scan_merged.ranges = modified_ranges
        

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
                    
    def draw_object_on_laser_scan(self, msg, local_map, x, y):
        # Convert object position to polar coordinates
        angle = math.atan2(y, x)
        distance = math.sqrt(x**2 + y**2)
        
        # use local map to find which indexes to change
        # Find the range of angles that correspond to the object
        half_w = self.object_width / 2
        half_h = self.object_height / 2

        # Calculate the angular extent of the object
        angle_to_left_edge = math.atan2(y + half_h, x - half_w)
        angle_to_right_edge = math.atan2(y - half_h, x + half_w)

        # Find corresponding indices in the laser scan
        angle_min_idx = int((angle_to_right_edge - msg.angle_min) / msg.angle_increment)
        angle_max_idx = int((angle_to_left_edge - msg.angle_min) / msg.angle_increment)

        # Clamp indices to valid range
        angle_min_idx = max(0, min(angle_min_idx, len(msg.ranges) - 1))
        angle_max_idx = max(0, min(angle_max_idx, len(msg.ranges) - 1))

        # Create a modified laser scan with the object's distance
        modified_ranges = list(msg.ranges)
        for i in range(angle_min_idx, angle_max_idx + 1):
            if distance < modified_ranges[i] or modified_ranges[i] < msg.range_min or modified_ranges[i] > msg.range_max:
                modified_ranges[i] = distance

        return modified_ranges

    def publish_map(self):
        if self.local_map is None:
            return 
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = 'base_link'

        occupancy_grid.info.resolution = self.map_resolution
        occupancy_grid.info.width = self.map_width_cells
        occupancy_grid.info.height = self.map_height_cells
        occupancy_grid.info.origin.position.x = -self.map_width / 2
        occupancy_grid.info.origin.position.y = -self.map_height / 2
        occupancy_grid.info.origin.orientation.w = 1.0

        occupancy_grid.data = self.local_map.flatten().tolist()
        self.map_publisher.publish(occupancy_grid)
        
        if self.laser_scan_merged is not None:
            self.laser_scan_merged.header.stamp = self.get_clock().now().to_msg()
            self.laser_scan_merged.header.frame_id = 'base_link'
            self.local_map_scan_publisher.publish(self.laser_scan_merged)


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
