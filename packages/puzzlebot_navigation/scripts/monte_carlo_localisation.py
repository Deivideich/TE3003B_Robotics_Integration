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
from bisect import bisect_left

class MCLNode(Node):
    def __init__(self):
        super().__init__('mcl_node')

        self.declare_parameter('isDebug', False)

        self.num_particles = 100
        self.particles = []        
        self.particle_weights = np.zeros(self.num_particles)
        
        self.map = None
        self.map_received = False

        self.last_odom = None
        self.last_odom = None
        self.odom_received = False
        self.odom_covariance = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
        self.delta_motion = []
        
        self.last_scan = None
        self.scan_received = False        

        self.min_distance = 0.1
        self.min_angle = 10*math.pi/180.0
        self.predictionCounter = 0
        self.m_sync_count =0
        self.repropagateCountNeeded = 1

        #### TF HANDLERS ####
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        #### PUBLISHERS ####
        self.scan_pub = self.create_publisher(LaserScan, '/scan_view', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/mcl_pose', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/particle_cloud', 10)
        
        #### SUBSCRIBERS ####
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        #### TIMER ####
        self.timer = self.create_timer(0.1, self.mcl_loop)

        ### DEBUG ####
        if self.get_parameter('isDebug').get_parameter_value().bool_value:
            self.get_logger().info("Debug mode is ON")
            self.create_timer(0.1, self.publish_real_pose)
            

    # def publish_real_pose(self):

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
            # self.get_logger().info(f"Particle: x={x}, y={y}, theta={theta}")
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
        try:
            return tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        except Exception as e:
            self.get_logger().warn(f"Error converting euler to quaternion: {str(e)}")
            return (0.0, 0.0, 0.0, 1.0)
    
    def scan_callback(self, msg):
        self.scan = msg
        self.scan_received = True

        scan_msg = msg
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the scan message
        self.scan_pub.publish(scan_msg)

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_received = True
        if self.last_odom is None:
            self.last_odom = msg
            return
        # Save delta odom
        self.delta_motion = self.compute_odometry_delta(self.last_odom, self.odom)

    ## TODO: CHECK THIS METHOD
    def sensor_update(self):
        if not self.scan_received:
            return

        scan_angles = np.arange(self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment)
        scan_ranges = np.array(self.scan.ranges)
        max_range = self.scan.range_max
        maxScore = 0.0
        
        try:
            for j, particle in enumerate(self.particles):
                x, y, theta = particle
                weight = 0.0
                #theta = theta*np.pi/180.0

                for i in range(0, len(scan_angles), 1):  # Check every ~20th beam for speed
                    angle = scan_angles[i]
                    r = scan_ranges[i]
                    if r >= max_range or np.isnan(r):
                        continue
                        
                    # Transform laser point to world
                    beam_x = x + r * np.cos(theta + angle)
                    beam_y = y + r * np.sin(theta + angle)

                    float_x = (beam_x - self.map_origin.x) / self.map_resolution
                    float_y = (beam_y - self.map_origin.y) / self.map_resolution

                    # Check if the BEAM is inf or nan
                    if np.isnan(float_x) or np.isnan(float_y) or np.isinf(float_x) or np.isinf(float_y):
                        continue

                    # Convert world -> map indices
                    map_x = int((beam_x - self.map_origin.x) / self.map_resolution)
                    map_y = int((beam_y - self.map_origin.y) / self.map_resolution)

                    if not(0 <= map_x < self.map_width and 0 <= map_y < self.map_height):
                        continue
                    # Check if the be
                
                    weight += self.map_array[map_y, map_x] / 255.0
                
                self.particle_weights[j] += weight / len(scan_angles)

                if maxScore < weight:
                    maxScore = weight
                    self.maxParticleTuple = particle
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
            # If the transform fails, we can skip this particle
            # or handle it in a way that makes sense for your application
            # For now, we'll just continue to the nex


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

    def estimate_pose(self):
        if hasattr(self, 'maxParticleTuple'):
            return self.maxParticleTuple
        return np.array([0.0, 0.0, 0.0])
    
    def resample_particles(self):
        particlesScores = []
        particleSampled = []
        scoreBase = 0.0

        for i in range(self.num_particles):
            scoreBase += self.particle_weights[i]
            particlesScores.append(scoreBase)

        for _ in range(self.num_particles):
            randomNum = np.random.uniform(0, scoreBase)
            index = bisect_left(particlesScores, randomNum)

            particle = self.particles[index]
            particleSampled.append(particle)
        
        self.particles = []
        for _ in range(self.num_particles):
            particle = particleSampled[np.random.randint(0, len(particleSampled))]
            x_offset = np.random.normal(0, 0.05)
            y_offset = np.random.normal(0, 0.05)
            theta_offset = np.random.normal(-np.pi/16, np.pi/16)
            x_new = particle[0] + x_offset
            y_new = particle[1] + y_offset
            theta_new = particle[2] + theta_offset
            self.particles.append((x_new, y_new, theta_new))
            
        self.particles = np.array(self.particles)
        self.particle_weights = np.zeros(self.particles.shape[0])

    
    ## TODO: CHECK THIS METHOD
    def motion_update(self, delta):
        dx, dy, dtheta = delta

        delta_trans = math.sqrt(dx**2 + dy**2)
        delta_rot = math.atan2(dy, dx)
        
        trans_noise_coeff = self.odom_covariance[2] * abs(delta_trans) + self.odom_covariance[3] * abs(dtheta)
        rot_noise_coeff = self.odom_covariance[0] * abs(dtheta) + self.odom_covariance[1] * abs(delta_trans)

        for i, (x, y, theta) in enumerate(self.particles):
            delta_rot1 = self.angle_diff(delta_rot, theta)
            delta_rot2 = self.angle_diff(dtheta, delta_rot1)

            delta_trans_noisy = delta_trans + np.random.normal(0, trans_noise_coeff)
            delta_rot1_noisy = delta_rot1 + np.random.normal(0, rot_noise_coeff)
            delta_rot2_noisy = delta_rot2 + np.random.normal(0, rot_noise_coeff)

            x_new = x + delta_trans_noisy * math.cos(theta + delta_rot1_noisy)
            y_new = y + delta_trans_noisy * math.sin(theta + delta_rot1_noisy)
            theta_new = theta + delta_rot1_noisy + delta_rot2_noisy

            self.particles[i] = (x_new, y_new, theta_new)
            
    def broadcast_transform(self):
        try:
            # Get latest odom -> base_link transform
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            
            # Get estimated pose in map (from particle cloud)
            x, y, theta = self.estimate_pose()

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
            #self.get_logger().info(f"Transform from odom to base_footprint: trans={trans.transform.translation}, rot={trans.transform.rotation}")
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


    def mcl_loop(self):
        if self.map is None:
            return
        if self.particles is None or len(self.particles) == 0:
            return
        if len(self.delta_motion) <= 0:
            return
        
        diffDistance = math.sqrt(self.delta_motion[0]**2 + self.delta_motion[1]**2)
        diffAngle = abs(self.delta_motion[2])*180.0/3.141592
        # self.get_logger().info(f"MCL: distance={diffDistance}, angle={diffAngle}")
        if not(diffDistance < self.min_distance and diffAngle < self.min_angle):
            # self.get_logger().info(f"Updating particles")
            self.motion_update(self.delta_motion)       
            self.sensor_update()

            self.predictionCounter += 1
            neff = 1.0 / np.sum(np.square(self.particle_weights))

            if (neff < self.num_particles / 2) and (self.predictionCounter == self.repropagateCountNeeded):
                self.resample_particles()
                self.predictionCounter = 0
            
            self.last_odom = self.odom
            
        self.publish_particles()
        self.broadcast_transform()
        self.publish_estimated_pose()


def main(args=None):
    np.seterr(over='raise')
    rclpy.init(args=args)
    node = MCLNode()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()

