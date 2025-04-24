#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf_transformations

class MCLNode(Node):
    def __init__(self):
        super().__init__('mcl_node')

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'mcl_pose', 10)

        self.num_particles = 100
        self.particles = []
        self.particles_pub = self.create_publisher(PoseArray, 'particle_cloud', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.particle_weights = np.zeros(self.num_particles)
        self.last_odom = None
        self.map = None
        self.last_scan = None
        self.last_odom = None
        self.map_received = False
        self.timer = self.create_timer(0.1, self.mcl_loop)
        self.scan_received = False
        self.odom_received = False

        self.tf_broadcaster = TransformBroadcaster(self)

    def publish_estimated_pose(self):
        est = self.estimate_pose()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = est[0]
        msg.pose.pose.position.y = est[1]
        msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, est[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # You can tweak this covariance as needed
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[35] = 0.2

        self.pose_pub.publish(msg)

    def map_callback(self, msg):    
        self.map = msg

        if not self.map_received:
            self.get_logger().info("Map received")
            self.map_received = True

            # Get map properties
            self.map_resolution = msg.info.resolution
            self.map_origin = msg.info.origin.position
            self.map_width = msg.info.width
            self.map_height = msg.info.height

            # Convert to 2D array
            map_data = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
            self.map_array = map_data

            self.initialize_particles()

            
    def initialize_particles(self):
        self.get_logger().info(f"Initializing {self.num_particles} particles")
        if len(self.particles) == 0:
            self.get_logger().warn("No particles initialized!")
        map_data = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))
        resolution = self.map.info.resolution
        origin = self.map.info.origin

        free_indices = np.argwhere(map_data == 0)  # 0 = free space

        chosen_indices = free_indices[np.random.choice(len(free_indices), self.num_particles)]

        self.particles = []
        for idx in chosen_indices:
            y_idx, x_idx = idx
            x = x_idx * resolution + origin.position.x
            y = y_idx * resolution + origin.position.y
            theta = np.random.uniform(-np.pi, np.pi)
            self.particles.append((x, y, theta))

        self.publish_particles()
    
    
    def publish_particles(self):
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'  # This should match your fixed frame in RViz

        for x, y, theta in self.particles:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0

            q = self.euler_to_quaternion(0, 0, theta)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            pa.poses.append(pose)

        self.particles_pub.publish(pa)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Returns (x, y, z, w)
        return tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    
    def scan_callback(self, msg):
        self.scan = msg
        self.scan_received = True


    def odom_callback(self, msg):
        self.odom = msg
        self.odom_received = True

        # Save delta odom
        if self.last_odom is not None:
            self.delta_motion = self.compute_odometry_delta(self.last_odom, self.odom)
        self.last_odom = self.odom

    def sensor_update(self):
        if not self.scan_received:
            return

        scan_angles = np.arange(self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment)
        scan_ranges = np.array(self.scan.ranges)
        max_range = self.scan.range_max

        weights = []

        for particle in self.particles:
            x, y, theta = particle
            weight = 1.0

            for i in range(0, len(scan_angles), 1):  # Check every ~20th beam for speed
                angle = scan_angles[i]
                r = scan_ranges[i]

                if r >= max_range or np.isnan(r):
                    continue

                # Transform laser point to world
                beam_x = x + r * np.cos(theta + angle)
                beam_y = y + r * np.sin(theta + angle)

                # Convert world -> map indices
                map_x = int((beam_x - self.map_origin.x) / self.map_resolution)
                map_y = int((beam_y - self.map_origin.y) / self.map_resolution)

                if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                    cell = self.map_array[map_y, map_x]
                    # If beam hits occupied space → higher weight
                    
                    if cell > 50:
                        weight *= 1.8  # High weight for hitting obstacle
                    elif cell == 0:
                        weight *= 0.5  # Penalize for hitting free space
                    else:
                        weight *= 0.1  # Unknown or out-of-bounds

                else:
                    weight *= 0.1  # Out of bounds

            weights.append(weight)

        # Normalize weights
        weights = np.array(weights)
        weights += 1e-300  # Avoid divide by zero
        weights /= np.sum(weights)

        self.particle_weights = weights


    def compute_odometry_delta(self, last_odom, current_odom):
        def get_pose(odom):
            pos = odom.pose.pose.position
            ori = odom.pose.pose.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            return pos.x, pos.y, yaw

        x1, y1, theta1 = get_pose(last_odom)
        x2, y2, theta2 = get_pose(current_odom)

        dx = x2 - x1
        dy = y2 - y1
        dtheta = self.angle_diff(theta2, theta1)
        return dx, dy, dtheta
    
    def angle_diff(self, a, b):
        diff = a - b
        return (diff + np.pi) % (2 * np.pi) - np.pi

    def mcl_loop(self):
        if self.map is None or self.particles is None or len(self.particles) == 0:
            return

        if hasattr(self, 'delta_motion') and ( math.sqrt(self.delta_motion[0]**2 + self.delta_motion[1]**2) > 0.001 or abs(self.delta_motion[2]) > 0.001):
            # Update particles based on odometry
            self.get_logger().info(f"UPDATING PARTICLES")
            self.motion_update(self.delta_motion)       
            self.sensor_update()

            neff = 1.0 / np.sum(np.square(self.particle_weights))
            if neff < self.num_particles / 2:
                self.resample_particles()
            
        self.publish_particles()
        self.broadcast_transform()
        self.publish_estimated_pose()


    def estimate_pose(self):
        x = 0.0
        y = 0.0
        sin_sum = 0.0
        cos_sum = 0.0

        for i, (px, py, theta) in enumerate(self.particles):
            weight = self.particle_weights[i]
            x += px * weight
            y += py * weight
            sin_sum += np.sin(theta) * weight
            cos_sum += np.cos(theta) * weight

        theta = np.arctan2(sin_sum, cos_sum)
        return np.array([x, y, theta])
    
    def resample_particles(self):
        new_particles = []
        M = self.num_particles
        weights = self.particle_weights

        index = int(np.random.rand() * M)
        beta = 0.0
        mw = np.max(weights)

        for _ in range(M):
            beta += np.random.rand() * 2.0 * mw
            while beta > weights[index]:
                beta -= weights[index]
                index = (index + 1) % M
            new_particles.append(self.particles[index])

        self.particles = np.array(new_particles)
    
    def motion_update(self, delta):
        dx, dy, dtheta = delta

        motion_noise = {
            "x": 0.01,
            "y": 0.01,
            "theta": 0.01
        }
        alpha1 = 0.05  # noise related to translational motion
        alpha2 = 0.01  # noise related to rotational motion 
        sigma_x = alpha1 * abs(dx) + alpha2 * abs(dtheta)
        sigma_y = alpha1 * abs(dy) + alpha2 * abs(dtheta)
        sigma_theta = alpha2 * abs(dtheta) + alpha1 * (abs(dx) + abs(dy))


        new_particles = []
        for x, y, theta in self.particles:
            # Apply rotation to delta to account for current particle heading
            dx_world = dx * math.cos(theta) - dy * math.sin(theta)
            dy_world = dx * math.sin(theta) + dy * math.cos(theta)

            x_new = x + dx_world + np.random.normal(0, sigma_x)
            y_new = y + dy_world + np.random.normal(0, sigma_y)
            theta_new = theta + dtheta + np.random.normal(0, sigma_theta)
            theta_new = self.angle_diff(theta_new, 0)  # Normalize

            new_particles.append((x_new, y_new, theta_new))

        self.particles = new_particles
            
    def broadcast_transform(self):
        try:
            # Get latest odom -> base_link transform
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            
            # Get estimated pose in map (from particle cloud)
            x, y, theta = self.estimate_pose()
            q_map = self.euler_to_quaternion(0, 0, theta)

            # Compose transform from map -> base_link
            T_map_base = tf_transformations.compose_matrix(
                translate=[x, y, 0],
                angles=[0, 0, theta]
            )

            # Compose transform from odom -> base_link
            trans_t = trans.transform.translation
            trans_q = trans.transform.rotation
            T_odom_base = tf_transformations.compose_matrix(
                translate=[trans_t.x, trans_t.y, trans_t.z],
                angles=tf_transformations.euler_from_quaternion([trans_q.x, trans_q.y, trans_q.z, trans_q.w])
            )

            # map -> odom = map -> base_link × inverse(odom -> base_link)
            # Note: We apply inverse of odom -> base_link first
            self.get_logger().info(f"Transform from odom to base_footprint: trans={trans.transform.translation}, rot={trans.transform.rotation}")
            T_map_odom = np.matmul(T_map_base, np.linalg.inv(T_odom_base))
            
            # Extract translation and rotation
            trans = tf_transformations.translation_from_matrix(T_map_odom)
            rot = tf_transformations.quaternion_from_matrix(T_map_odom)

            # Create TransformStamped message for map -> odom
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'

            # Fill in translation and rotation for map -> odom transform
            t.transform.translation.x = trans[0]
            t.transform.translation.y = trans[1]
            t.transform.translation.z = trans[2]
            t.transform.rotation.x = rot[0]
            t.transform.rotation.y = rot[1]
            t.transform.rotation.z = rot[2]
            t.transform.rotation.w = rot[3]

            # Publish the transform
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = MCLNode()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()

