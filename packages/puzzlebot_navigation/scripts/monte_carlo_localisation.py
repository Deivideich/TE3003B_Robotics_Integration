#!/usr/bin/env python3
import math
import numpy as np
import ctypes
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped, PoseArray, PoseStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
import tf_transformations

import os
import ament_index_python.packages

package_prefix = ament_index_python.packages.get_package_prefix('puzzlebot_navigation')
cpp_mcl = os.path.join(package_prefix, 'lib', 'puzzlebot_navigation', 'mcl_utils.so')

# cpp_mcl = "/workspace/8voSemestre/TE3003B_Robotics_Integration/packages/puzzlebot_navigation/utils/cpp/mcl_utils.so"


class MCLNode(Node):
    def __init__(self):
        super().__init__('mcl_node')
        self.mcl_cpp = ctypes.CDLL(cpp_mcl)

        self.declare_parameter('isDebug', False)
        self.declare_parameter('useClustering', True)

        self.num_particles = 1000
        self.num_dimensions = 3
        self.particles = []        
        self.particle_weights = np.zeros(self.num_particles)
        self.cluster_dbscan = DBSCAN(eps=0.5, min_samples=int(self.num_particles * 0.05), metric='euclidean', n_jobs=-1)
        
        self.map = None
        self.map_received = False

        self.last_odom = None
        self.last_odom = None
        self.odom_received = False
        self.odom_covariance = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
        self.delta_motion = []
        
        self.last_scan = None
        self.scan_received = False        

        self.min_distance = 0.05
        self.min_angle = 10
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
        self.timer = self.create_timer(0.05, self.mcl_loop)

        ### DEBUG ####
        if self.get_parameter('isDebug').get_parameter_value().bool_value:
            self.get_logger().info("Debug mode is ON")
            self.create_timer(0.1, self.publish_real_pose)


    def publish_estimated_pose(self):
        x, y, theta = self.estimate_pose()

        x, y, theta = float(x), float(y), float(theta)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, theta)
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
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
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


    def sensor_update(self):
        try:
            if not self.scan_received:
                return
            
            map_array = np.array(self.map.data, dtype=np.int32)
            map_origin = np.array([self.map_origin.x, self.map_origin.y], dtype=np.float32)
            map_shape = np.array([self.map_height, self.map_width], dtype=np.int32)
            scan_angles = np.arange(self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment, dtype=np.float32)
            scan_ranges = np.array(self.scan.ranges)
            max_range = self.scan.range_max

            assert len(scan_ranges) == len(scan_angles), "Scan ranges and angles must have the same size"
            
            particles = np.array(self.particles, dtype=np.float32).flatten()
            output_weights = np.zeros(self.num_particles, dtype=np.float32)
            max_particle = np.zeros(self.num_dimensions, dtype=np.float32)
        
            self.mcl_cpp.weight_particles.argtypes = [
                ctypes.POINTER(ctypes.c_int),                      # map_array
                ctypes.POINTER(ctypes.c_float),                      # map_origin
                ctypes.POINTER(ctypes.c_int),                    # map_shape
                ctypes.c_float,                    # map_resolution
                ctypes.POINTER(ctypes.c_float),    # scan_angles
                ctypes.POINTER(ctypes.c_float),    # scan_ranges
                ctypes.c_int,                      # scan_size
                ctypes.c_float,                    # max_range
                ctypes.c_int,                      # num_particles
                ctypes.c_int,                      # num_dimensions
                ctypes.POINTER(ctypes.c_float),    # particles
                ctypes.POINTER(ctypes.c_float),    # max_particles (OUTPUT)
                ctypes.POINTER(ctypes.c_float),    # weights (OUTPUT)
            ]
            self.mcl_cpp.weight_particles.restype = ctypes.c_bool

            # Convert numpy arrays to ctypes
            marray_ctypes = map_array.ctypes.data_as(ctypes.POINTER(ctypes.c_int))
            morigin_ctypes = map_origin.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            mshape_ctypes = map_shape.ctypes.data_as(ctypes.POINTER(ctypes.c_int))

            sangles_ctypes = scan_angles.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            sranges_ctypes = scan_ranges.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            
            particles_ctypes = particles.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            max_particle_ctypes = max_particle.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            out_weights = output_weights.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

            success = self.mcl_cpp.weight_particles(
                marray_ctypes, 
                morigin_ctypes,
                mshape_ctypes,
                self.map_resolution,
                sangles_ctypes,
                sranges_ctypes,
                len(scan_angles),
                max_range,
                self.num_particles,
                self.num_dimensions,
                particles_ctypes,
                max_particle_ctypes,
                out_weights,
            )

            if success:
                self.maxParticle = max_particle
                self.particle_weights = output_weights
            else:
                self.get_logger().warn("C++ resampling failed. Falling back to Python version.")

        except Exception as e:
            self.get_logger().warn(f"{str(e)}")


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

    #TODO: this function is good but slow, DBSSCAN compute wise is not efficient, need to find a better way to cluster
    def estimate_pose(self):
        if self.get_parameter('useClustering').get_parameter_value().bool_value:
            clusters = self.cluster_dbscan.fit(self.particles)
            unique_labels = set(clusters.labels_)
            if -1 in unique_labels:
                unique_labels.remove(-1)
            if len(unique_labels) == 0:
                if hasattr(self, 'maxParticle'):
                    return np.array([float(self.maxParticle[0]), float(self.maxParticle[1]), float(self.maxParticle[2])])
                else: 
                    return np.array([0.0, 0.0, 0.0])
                
            maxParticle = self.maxParticle if hasattr(self, 'maxParticle') else np.array([0.0, 0.0, 0.0])
            minDistance = 1000000
            bestCluster = None
            # Look for maxParticle in clusters
            for label in unique_labels:
                if label == -1:
                    continue
                cluster_indices = np.where(clusters.labels_ == label)[0]
                cluster_particles = [self.particles[i] for i in cluster_indices]
                cluster_center = np.mean(cluster_particles, axis=0)
                if np.linalg.norm(cluster_center - maxParticle) < minDistance:
                    minDistance = np.linalg.norm(cluster_center - maxParticle)
                    bestCluster = np.mean(cluster_particles, axis=0)
                    self.get_logger().info(f"Cluster center: {cluster_center}")
            
            return bestCluster if bestCluster is not None else maxParticle     
        if hasattr(self, 'maxParticle'):
            return np.array([float(self.maxParticle[0]), float(self.maxParticle[1]), float(self.maxParticle[2])])
        return np.array([0.0, 0.0, 0.0])
    
    def resample_particles(self):
        self.get_logger().info("Resampling particles")
        # Prepare arguments
        weights = self.particle_weights.astype(np.float32)
        particles = np.array(self.particles, dtype=np.float32).flatten()
        resampled_particles = np.zeros_like(particles)
        
        # Define the function signature
        self.mcl_cpp.resample_particles.argtypes = [
            ctypes.c_int,                      # num_particles
            ctypes.c_int,                      # num_dimensions
            ctypes.c_float,                    # theta_noise
            ctypes.c_float,                    # trans_noise
            ctypes.POINTER(ctypes.c_float),    # weights
            ctypes.POINTER(ctypes.c_float),    # particles
            ctypes.POINTER(ctypes.c_float)     # resampled_particles
        ]
        self.mcl_cpp.resample_particles.restype = ctypes.c_bool

        # Convert numpy arrays to ctypes
        weights_ctypes = weights.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        particles_ctypes = particles.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        resampled_ctypes = resampled_particles.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

        # Call the function
        success = self.mcl_cpp.resample_particles(
            self.num_particles,
            self.num_dimensions,
            np.float32((np.pi / 32)),
            np.float32(0.04),
            weights_ctypes,
            particles_ctypes,
            resampled_ctypes
        )

        if success:
            self.particles = resampled_particles.reshape((self.num_particles, 3))
            self.particle_weights = np.ones(self.num_particles)
            self.particle_weights /= self.num_particles
        else:
            self.get_logger().warn("C++ resampling failed. Falling back to Python version.")


    def motion_update(self, delta):
        dx, dy, dtheta = delta

        delta_trans = math.sqrt(dx**2 + dy**2)
        delta_rot = math.atan2(dy, dx)
        
        trans_noise_coeff = self.odom_covariance[2] * abs(delta_trans) + self.odom_covariance[3] * abs(dtheta)
        rot_noise_coeff = self.odom_covariance[0] * abs(dtheta) + self.odom_covariance[1] * abs(delta_trans)

        for i, (x, y, theta) in enumerate(self.particles):
            delta_rot1 = self.angle_diff(math.atan2(dy, dx), theta)
            delta_rot2 = self.angle_diff(dtheta, delta_rot1)

            delta_trans_noisy = delta_trans + np.random.normal(0, trans_noise_coeff)
            delta_rot1_noisy = delta_rot1 + np.random.normal(0, rot_noise_coeff)
            delta_rot2_noisy = delta_rot2 + np.random.normal(0, rot_noise_coeff)

            x_new = x + delta_trans_noisy * math.cos(theta + delta_rot1_noisy)
            y_new = y + delta_trans_noisy * math.sin(theta + delta_rot1_noisy)
            theta_new = theta + delta_rot1_noisy + delta_rot2_noisy
            theta_new = (theta_new + math.pi) % (2 * math.pi) - math.pi

            self.particles[i] = (x_new, y_new, theta_new)

    def broadcast_transform(self):
        try:
            x, y, theta = self.estimate_pose()

            x, y, theta = float(x), float(y), float(theta)

            # Get odom -> base_link transform
            trans = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Compose transformation: map -> base_link
            map_to_base = tf_transformations.compose_matrix(
                translate=[x, y, 0],
                angles=[0, 0, theta]
            )

            # Compose transformation: odom -> base_link (from TF)
            trans_translation = trans.transform.translation
            trans_rotation = trans.transform.rotation
            odom_to_base = tf_transformations.compose_matrix(
                translate=[trans_translation.x, trans_translation.y, trans_translation.z],
                angles=tf_transformations.euler_from_quaternion([
                    trans_rotation.x,
                    trans_rotation.y,
                    trans_rotation.z,
                    trans_rotation.w
                ])
            )

            # map -> odom = map -> base × inverse(odom -> base)
            base_to_odom = np.linalg.inv(odom_to_base)
            map_to_odom = np.dot(map_to_base, base_to_odom)

            translation = map_to_odom[:3, 3]
            rotation = tf_transformations.quaternion_from_matrix(map_to_odom)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'odom'
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            t.transform.rotation.x = rotation[0]
            t.transform.rotation.y = rotation[1]
            t.transform.rotation.z = rotation[2]
            t.transform.rotation.w = rotation[3]

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f"TF broadcast error: {str(e)}")


    def mcl_loop(self):
        if self.map is None:
            return
        if self.particles is None or len(self.particles) == 0:
            return
        if len(self.delta_motion) <= 0:
            return
        
        diffDistance = math.sqrt(self.delta_motion[0]**2 + self.delta_motion[1]**2)
        diffAngle = abs(self.delta_motion[2])*180.0/3.141592

        if diffDistance > self.min_distance or diffAngle > self.min_angle:
            self.motion_update(self.delta_motion)       
            self.last_odom = self.odom
            
            self.sensor_update()
            
            self.predictionCounter += 1
            neff = 1.0 / np.sum(np.square(self.particle_weights))

            if (neff > self.num_particles * 0.1) and (self.predictionCounter >= self.repropagateCountNeeded):
                self.resample_particles()
                self.predictionCounter = 0
        

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

