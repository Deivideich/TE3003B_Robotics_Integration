#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np



class LocalMapPublisher(Node):
    def __init__(self):
        super().__init__('local_map_publisher')

        # Parameters for map size and resolution
        self.declare_parameter('map_width', 2.0)  # Width in meters
        self.declare_parameter('map_height', 2.0)  # Height in meters
        self.declare_parameter('map_resolution', 0.05)  # Resolution in meters per cell

        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_resolution = self.get_parameter('map_resolution').value

        # Derived parameters
        self.map_width_cells = int(self.map_width / self.map_resolution)
        self.map_height_cells = int(self.map_height / self.map_resolution)

        # Publisher for the local map
        self.map_publisher = self.create_publisher(OccupancyGrid, 'local_map', 10)

        # Subscriber for LaserScan data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Local map publisher node initialized.')

    def scan_callback(self, msg: LaserScan):
        # Create an empty map (all cells unknown, represented by -1)
        local_map = -1 * np.ones((self.map_height_cells, self.map_width_cells), dtype=np.int8)

        # Convert LaserScan data to map coordinates
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Calculate the position in meters
                x = r * np.cos(angle)
                y = r * np.sin(angle)

                # Convert to map grid coordinates
                map_x = int((x + self.map_width / 2) / self.map_resolution)
                map_y = int((y + self.map_height / 2) / self.map_resolution)

                # Check bounds and mark the cell as occupied (100)
                if 0 <= map_x < self.map_width_cells and 0 <= map_y < self.map_height_cells:
                    local_map[map_y, map_x] = 100

            angle += msg.angle_increment

        # Publish the map as an OccupancyGrid
        self.publish_map(local_map)

    def publish_map(self, local_map):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = 'base_link'

        occupancy_grid.info.resolution = self.map_resolution
        occupancy_grid.info.width = self.map_width_cells
        occupancy_grid.info.height = self.map_height_cells
        occupancy_grid.info.origin.position.x = -self.map_width / 2
        occupancy_grid.info.origin.position.y = -self.map_height / 2
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0

        # Flatten the map and assign it to the data field
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