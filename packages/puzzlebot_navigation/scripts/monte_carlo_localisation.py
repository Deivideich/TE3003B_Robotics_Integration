#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import tf_transformations
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import math

class MCLNode(Node):
    def __init__(self):
        super().__init__('mcl_node')
        self.num_particles = 100
        self.declare_parameter('wait_for_map', True)
        self.wait_for_map = self.get_parameter('wait_for_map').get_parameter_value().bool_value
        self.map = None
        self.last_odom = None
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Optional: Wait for the map
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        if self.wait_for_map:
            while rclpy.ok() and self.map is None:
                self.get_logger().info('Waiting for /map...')
                rclpy.spin_once(self, timeout_sec=1.0)

        self.particles = self.initialize_particles_from_map()


        self.particle_pub = self.create_publisher(PoseArray, '/particle_cloud', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer to publish particles every second
        self.create_timer(1.0, self.publish_particles)

    def scan_callback(self, msg):
        self.scan = msg
        self.sensor_update()

    def quaternion_to_yaw(self, orientation_q):
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        return yaw

    def sensor_update(self):
        if self.scan is None or self.map is None:
            return

        weights = []

        angles = np.arange(self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment)
        scan_ranges = np.array(self.scan.ranges)

        # Pick fewer rays for faster processing
        ray_indices = np.arange(0, len(angles), 20)

        for p in self.particles:
            px, py, ptheta = p
            weight = 0.0

            for i in ray_indices:
                angle = angles[i]
                dist = scan_ranges[i]

                if np.isinf(dist) or np.isnan(dist):
                    continue

                # Simulate ray endpoint from this particle
                map_x = px + dist * np.cos(ptheta + angle)
                map_y = py + dist * np.sin(ptheta + angle)

                if self.is_occupied(map_x, map_y):
                    weight += 1.0  # Hit something where we expected
                else:
                    weight += 0.1  # Empty or unknown

            weights.append(weight)

        weights = np.array(weights)
        weights += 1e-300  # prevent divide by zero
        weights /= np.sum(weights)

        # Resample particles based on weights
        indices = np.random.choice(len(self.particles), size=self.num_particles, p=weights)
        self.particles = self.particles[indices]

    def odom_callback(self, msg):
        # Get current odometry pose
        current_pos = msg.pose.pose.position
        current_ori = msg.pose.pose.orientation
        current_theta = self.quaternion_to_yaw(current_ori)

        if self.last_odom is None:
            self.last_odom = (current_pos.x, current_pos.y, current_theta)
            return

        # Calculate robot's movement since last time
        last_x, last_y, last_theta = self.last_odom
        dx = current_pos.x - last_x
        dy = current_pos.y - last_y
        dtheta = current_theta - last_theta

        # Compute movement in robot's local frame
        local_dx = np.cos(-last_theta) * dx - np.sin(-last_theta) * dy
        local_dy = np.sin(-last_theta) * dx + np.cos(-last_theta) * dy

        # Move particles with added noise
        for i, p in enumerate(self.particles):
            x, y, theta = p

            # Add motion + noise
            noisy_dx = local_dx + np.random.normal(0, 0.01)
            noisy_dy = local_dy + np.random.normal(0, 0.01)
            noisy_dtheta = dtheta + np.random.normal(0, 0.01)

            # Move particle
            new_x = x + np.cos(theta) * noisy_dx - np.sin(theta) * noisy_dy
            new_y = y + np.sin(theta) * noisy_dx + np.cos(theta) * noisy_dy
            new_theta = theta + noisy_dtheta

            self.particles[i] = np.array([new_x, new_y, new_theta])

        # Update odometry
        self.last_odom = (current_pos.x, current_pos.y, current_theta)

    def is_occupied(self, x, y):
        if self.map is None:
            return False

        mx = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((y - self.map.info.origin.position.y) / self.map.info.resolution)

        if 0 <= mx < self.map.info.width and 0 <= my < self.map.info.height:
            index = my * self.map.info.width + mx
            return self.map.data[index] > 50  # Occupied
        else:
            return False

    def initialize_particles(self):
        particles = []
        for _ in range(self.num_particles):
            x = np.random.uniform(0, 5.0)       # Map bounds (adjust!)
            y = np.random.uniform(0, 5.0)
            theta = np.random.uniform(-np.pi, np.pi)
            particles.append(np.array([x, y, theta]))
        return np.array(particles)

    def publish_particles(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            q = tf_transformations.quaternion_from_euler(0, 0, p[2])
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            msg.poses.append(pose)

        self.particle_pub.publish(msg)

    def initialize_particles_from_map(self):
        if self.map is None:
            self.get_logger().warn('Map not received yet!')
            return []

        particles = []
        map_data = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        resolution = self.map.info.resolution
        origin = self.map.info.origin

        while len(particles) < self.num_particles:
            # Randomly sample an index in the map
            i = np.random.randint(0, self.map.info.height)
            j = np.random.randint(0, self.map.info.width)

            # Only accept free space (value == 0)
            if map_data[i, j] == 0:
                x = origin.position.x + j * resolution
                y = origin.position.y + i * resolution
                theta = np.random.uniform(-np.pi, np.pi)
                particles.append(np.array([x, y, theta]))

        return np.array(particles)

    def map_callback(self, msg):
        self.map = msg


def main(args=None):
    rclpy.init(args=args)
    node = MCLNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()