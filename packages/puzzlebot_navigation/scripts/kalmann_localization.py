#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Int32, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
import cv2
import tf_transformations
from tf2_ros import TransformBroadcaster, TransformStamped, Buffer, TransformListener
import tf2_ros
import time
from rclpy.time import Time
from transforms3d.quaternions import quat2mat
from visualization_msgs.msg import Marker

class KalmanNode(Node):
    def __init__(self):
        super().__init__('kalman_node')
        
        self.last_estimated_pose_time = time.time()
        self.publish_estimated_pose_interval = 5  # seconds
        
        self.aruco_diff_x = 99.0
        self.aruco_diff_y = 99.0
        self.aruco_diff_angle = 99.0
        
        self.tf_mutex = False
        
        # create broadcast tf param
        self.declare_parameter('broadcast_tf', True)
        self.declare_parameter('mcl_aid', False)
        self.broadcast_tf = self.get_parameter('broadcast_tf').get_parameter_value().bool_value
        self.mcl_aid = self.get_parameter('mcl_aid').get_parameter_value().bool_value
        self.initial_pose = False

        # SUBSCRIBERS
        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, qos)
        self.create_subscription(Int32, '/aruco_id', self.aruco_callback, qos) #Subscriber del ARUCO id
        # Aqui tiene que ir el subscriber que me de la posicion de los marcadores

        #PUBLISHERS
        # self.pub_pos = self.create_publisher(PoseStamped, '/estimated_pose', 10)
        self.pub_pos = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)
        self.inital_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf_mutex_pub = self.create_publisher(Bool, '/ekf_tf_mutex', 10)
        
        # debug marker publisher to see projected landmarks
        self.debug_aruco_marker_pub = self.create_publisher(Marker, '/debug_aruco', 10)
        self.debug_aruco_obs_marker_pub = self.create_publisher(Marker, '/debug_aruco_obs', 10)

        self.timer = self.create_timer(0.02, self.timer_callback)

        #Variables for Dead Reckoning
        self.wheel_radius = 0.05
        self.wheel_base = 0.168 #0.168?

        self.omega_l = 0.0
        self.omega_r = 0.0

        self.dt = 0.0
        self.last_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                         self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        #Variables for Kalman
        self.Kalmann_gain = np.zeros((3, 3)) 
        self.uPose = np.zeros((3, 1))
        self.identity = np.eye(3) # Identity matrix
        self.gradient_H = np.zeros((3,3))
        
        self.uHat = np.zeros((3, 1))
        self.theta_prev = 0.0

        self.Sigma_cov = np.zeros((3,3))
        self.Sigma_hat = np.zeros((3,3))

        self.zHat = np.zeros((3, 1))

        self.Z_mat = np.zeros((3,3))

        self.gradient_G = np.zeros((3,3))
        self.gradient_G[1, 2] = -1
        
        #Q error for model
        self.error_Q = np.zeros((3,3)) # Process noise covariance matrix
        self.K_R = 0.3 # 0.30406416057210744
        self.K_L = 0.4 # 0.38899148975615183
        self.K_RB = 0.011
        self.K_LB = 0.002
        #Camera error, not tuned
        self.R_error = np.array([[0.00000001, 0, 0],
                                 [0, 0.00000001, 0],
                                 [0, 0, 0.000001]]) # Measurement noise covariance matrix
                
        #FLAGS FOR SUB CALLBACKS
        self.landmark_status = False
        self.new_odom = False
        self.valid_id = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] # Valid ARUCO IDs

        #TF HANDLERS
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        time.sleep(1)

    def obtain_Q(self):
        delta_w = np.zeros((3,2))
        delta_w[0,0] = np.cos(self.theta_prev)
        delta_w[0,1] = np.cos(self.theta_prev)
        delta_w[1,0] = np.sin(self.theta_prev)
        delta_w[1,1] = np.sin(self.theta_prev)
        delta_w[2,0] = 2 / self.wheel_base
        delta_w[2,1] = -2 / self.wheel_base

        delta_w = 0.5 * self.wheel_radius * self.dt * delta_w

        sigma_ = np.zeros((2,2))
        sigma_[0,0] = self.K_R * abs(self.omega_r) + self.K_RB
        sigma_[1,1] = self.K_L * abs(self.omega_l) + self.K_LB

        self.error_Q = delta_w @ sigma_ @ delta_w.T

    def joint_state_callback(self, msg):
        # Extract wheel velocities from JointState message
        if len(msg.velocity) < 2:
            self.get_logger().warn("Received less than 2 wheel velocities!")
            return

        self.omega_l = msg.velocity[0]
        self.omega_r = msg.velocity[1]

        # Direct kinematics
        self.v = self.wheel_radius * (self.omega_r + self.omega_l) / 2
        self.w =( self.wheel_radius * (self.omega_r - self.omega_l) / self.wheel_base) * 1.0

        self.new_odom = True
    
    def calcMiuHat(self):

        # Update dt
        current_time = self.get_clock().now().seconds_nanoseconds()
        now = current_time[0] + current_time[1] * 1e-9
        self.dt = now - self.last_time
        self.last_time = now

        self.uHat[0, 0] += self.dt * self.v * math.cos(self.theta_prev)
        self.uHat[1, 0] += self.dt * self.v * math.sin(self.theta_prev) 
        self.uHat[2, 0] += self.dt * self.w
        self.uHat[2, 0] = (self.uHat[2, 0] + math.pi) % (2 * math.pi) - math.pi #Corregir theta

    def calc_Gradient_h(self):
        self.gradient_H[0,0] = 1
        self.gradient_H[1,1] = 1
        self.gradient_H[2,2] = 1
        self.gradient_H[0, 2] = -self.dt * self.v * math.sin(self.theta_prev)
        self.gradient_H[1, 2] = self.dt * self.v * math.cos(self.theta_prev)
        
    
    def calc_SigmaHat(self):
        self.Sigma_hat = self.gradient_H @ self.Sigma_cov @ self.gradient_H.T + self.error_Q

    def obtain_tfs(self):
        try:
            now = self.get_clock().now().to_msg()
            # move now 0.05 to the past
            now = rclpy.time.Time.from_msg(now) - rclpy.duration.Duration(seconds=0.1)
            now = now.to_msg()
            
            self.aruco_tf = self.tf_buffer.lookup_transform(
                    'map',  # target frame - map
                    f'aruco_{self.marker_id}',      # source frame - aruco
                    now,
                    rclpy.duration.Duration(seconds=0.05))  # TODO    

            self.aruco_map_angle = self.aruco_yaw(self.aruco_tf.transform.rotation)

            self.aruco_to_robot_tf = self.tf_buffer.lookup_transform(
                'base_link',           # target frame - base footprint
                f'aruco_{self.marker_id}_ob',      # source frame - aruco
                Time())  #TODO
            
            x = self.aruco_to_robot_tf.transform.translation.x
            y = self.aruco_to_robot_tf.transform.translation.y
            
            q = self.aruco_to_robot_tf.transform.rotation
            
            aruco_to_robot_angle = (math.atan2(y, x) + np.pi) % (2 * np.pi) - np.pi
            
            if abs(aruco_to_robot_angle) > 0.4:
                self.get_logger().warn(f"Aruco ROBOT angle is too large: {aruco_to_robot_angle}. Skipping update.")
                return False
            
            self.get_logger().info(f"Aruco ROBOT angle: {aruco_to_robot_angle}")
            
            self.aruco_rot_tf = self.tf_buffer.lookup_transform(
                'map',           # target frame - base footprint
                f'aruco_{self.marker_id}_ob',      # source frame - aruco
                Time())
            
            self.aruco_robot_angle = self.aruco_yaw(self.aruco_rot_tf.transform.rotation)
            
            aruco_map_x = self.aruco_tf.transform.translation.x
            aruco_map_y = self.aruco_tf.transform.translation.y
            aruco_map_angle = self.aruco_yaw(self.aruco_tf.transform.rotation)
            aruco_obs_x = self.aruco_rot_tf.transform.translation.x
            aruco_obs_y = self.aruco_rot_tf.transform.translation.y
            aruco_obs_angle = self.aruco_yaw(self.aruco_rot_tf.transform.rotation)
            
            self.aruco_diff_x = aruco_map_x - aruco_obs_x
            self.aruco_diff_y = aruco_map_y - aruco_obs_y
            self.aruco_diff_angle = aruco_map_angle - aruco_obs_angle
            
            
        except Exception as e:
            self.get_logger().warn(f'Error obtaining transforms: {str(e)}')
            return False
    
        return True
    
    # obtains aruco angle on "2D" -> between x and z axis -> y point up
    def aruco_yaw(self, rotation):
        try:
            quat = [rotation.x, rotation.y, rotation.z, rotation.w]
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(quat)
            return yaw
        except Exception as e:
            self.get_logger().warn(f"Error converting quaternion to euler: {str(e)}")
            return 0.0
    
    
    def Calc_zHat(self):
        # Nos falta la posicion de los landmarks en el mapa real.
        # Dependiendo del landmark que veamos

        self.m_x = self.aruco_tf.transform.translation.x
        self.m_y = self.aruco_tf.transform.translation.y

        diff_x = self.m_x - self.uHat[0, 0]
        diff_y = self.m_y - self.uHat[1, 0]
        
        self.zHat[0, 0] = np.sqrt( (diff_x)**2 + (diff_y)**2 )
        
        # angle made from quat to robot
        angle = math.atan2(diff_y, diff_x) - self.uHat[2, 0] #TODO
        
        aruco_angle = self.aruco_map_angle # - self.uHat[2, 0] #TODO
        aruco_angle = (aruco_angle + np.pi) % (2 * np.pi) - np.pi # Normalize angle to [-pi, pi]
        
        # self.get_logger().info(f"Aruco {self.marker_id} MAP angle: {aruco_angle}")
        
        self.zHat[1, 0] = (angle + np.pi) % (2 * np.pi) - np.pi
        self.zHat[2, 0] = aruco_angle

        # noise = np.random.normal(0, 0.1, size=self.zHat.shape)
        # self.zHat += noise 

    def calc_Gradient_g(self):
        #TODO

        diff_x = self.m_x - self.uHat[0, 0]
        diff_y = self.m_y - self.uHat[1, 0]
        
        sum_sq = diff_y ** 2 + diff_x ** 2
        
        self.gradient_G[0, 0] = -diff_x / np.sqrt(sum_sq)
        self.gradient_G[0, 1] = -diff_y / np.sqrt(sum_sq)
        self.gradient_G[0, 2] = 0.0

        self.gradient_G[1, 0] = diff_y / sum_sq
        self.gradient_G[1, 1] = -diff_x / sum_sq
        self.gradient_G[1, 2] = -1.0
        
        self.gradient_G[2, 0] = 0.0
        self.gradient_G[2, 1] = 0.0
        self.gradient_G[2, 2] = -1.0

    def Calc_Z(self):
        
        self.Z_mat = self.gradient_G @ self.Sigma_hat @ self.gradient_G.T + self.R_error
        
    def calc_KalmannGain(self):
        self.Kalmann_gain = self.Sigma_hat @ self.gradient_G.T @ np.linalg.inv(self.Z_mat)
        
    def euclidean_distance(self, x, y):
        return np.sqrt( x*x + y*y )
    
    def yaw_from_quaternion(self, q):
        try:
            quat = [q.x, q.y, q.z, q.w]
            _, _, yaw = tf_transformations.euler_from_quaternion(quat)
            return yaw
        except Exception as e:
            self.get_logger().warn(f"Error converting quaternion to euler: {str(e)}")

            return 0.0

    def calc_miu(self):
        z_vec = np.zeros((3, 1)) # This needs to be the SinglePoseMarker with Transforms. I need euclidean distance and angle from base_footprint I think?

        x = self.aruco_to_robot_tf.transform.translation.x
        y = self.aruco_to_robot_tf.transform.translation.y
        
        q = self.aruco_to_robot_tf.transform.rotation
        

        z_vec[0, 0] = self.euclidean_distance(x, y)
        z_vec[1, 0] = (math.atan2(y, x) + np.pi) % (2 * np.pi) - np.pi
        
        self.aruco_robot_angle # -= self.uHat[2, 0] #TODO
        z_vec[2, 0] = (self.aruco_robot_angle + np.pi) % (2 * np.pi) - np.pi #TODO
        # self.get_logger().info(f"Aruco {self.marker_id} ROBOT angle: {z_vec[2, 0]}")
        
        self.uPose = self.uHat + self.Kalmann_gain @ (z_vec - self.zHat)


    def euler_to_quaternion(self, roll, pitch, yaw):
        # Returns (x, y, z, w)
        try:
            return tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        except Exception as e:
            self.get_logger().warn(f"Error converting euler to quaternion: {str(e)}")
            return (0.0, 0.0, 0.0, 1.0)
        

    def calc_sigma(self):
        self.Sigma_cov = (self.identity - self.Kalmann_gain @ self.gradient_G) @ self.Sigma_hat

    def estimate_pose(self):
        return self.uPose[0, 0], self.uPose[1, 0], self.uPose[2, 0] # x,y,theta


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



    def set_previous(self):
        self.uHat[0, 0] = self.uPose[0, 0]
        self.uHat[1, 0] = self.uPose[1, 0]
        self.uHat[2, 0] = self.uPose[2, 0]
        self.theta_prev = self.uPose[2, 0]

        q = self.euler_to_quaternion(0.0, 0.0, self.uPose[2, 0])
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map' #TODO
        msg.pose.pose.position.x = float(self.uPose[0, 0])
        msg.pose.pose.position.y = float(self.uPose[1, 0])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        ros_cov = np.zeros(36)
        ros_cov[0] = self.Sigma_cov[0, 0] # x,x
        ros_cov[1] = self.Sigma_cov[0, 1] # x,y
        ros_cov[5] = self.Sigma_cov[0, 2] # x,theta
        ros_cov[6] = self.Sigma_cov[1, 0] # y,x
        ros_cov[7] = self.Sigma_cov[1, 1] # y,y
        ros_cov[11] = self.Sigma_cov[1, 2] # y, theta
        ros_cov[30] = self.Sigma_cov[2, 0] # theta,x
        ros_cov[31] = self.Sigma_cov[2, 1] # theta,y
        ros_cov[35] = self.Sigma_cov[2, 2] #theta,theta

        msg.pose.covariance = ros_cov

        std_x = np.sqrt(self.Sigma_cov[0, 0])
        std_y = np.sqrt(self.Sigma_cov[1, 1])
        std_theta = np.sqrt(self.Sigma_cov[2, 2])  # radians

        # Define your thresholds
        max_std_x = 0.1     # 20 cm
        max_std_y = 0.1     # 20 cm
        max_std_theta = np.deg2rad(5)  # ~10 degrees

        if (abs(self.aruco_diff_x) < 0.05 and abs(self.aruco_diff_y) < 0.05 and abs(self.aruco_diff_angle) < np.deg2rad(30)) and \
            (time.time() - self.last_estimated_pose_time > self.publish_estimated_pose_interval):
            self.initial_pose = True
            self.get_logger().info(f"Aruco {self.marker_id} is within acceptable range, sending estimated pose.")
            self.inital_pose_pub.publish(msg)
            self.last_estimated_pose_time = time.time()
        
        self.pub_pos.publish(msg) 
        
        
    def aruco_callback(self, msg):
        self.marker_id = msg.data #TODO?
        if(self.marker_id in self.valid_id):
            self.landmark_status = True
        else:
            self.landmark_status = False

    def Kalmann_filter(self):
        #Obtain covariance
        # self.get_logger().info('Obtaining Q')
        
        start_time = time.perf_counter()
        self.obtain_Q()
        # self.get_logger().info('Dead Reckoning')
        self.calcMiuHat()
        self.calc_Gradient_h()
        self.calc_SigmaHat()
        # Check if landmark is visible, to correct using observations
        if(self.landmark_status and self.obtain_tfs()):
            # self.get_logger().info('Correction')
            self.Calc_zHat()
            self.calc_Gradient_g()
            self.Calc_Z()
            self.calc_KalmannGain()
            # self.get_logger().info(f'Kalman Gain:\n{self.Kalmann_gain}')
            self.calc_miu()
            self.calc_sigma()  
            self.landmark_status = False
            if (self.mcl_aid):
                self.tf_mutex = True    
        # If landmark is not visible, use prediction (Dead Reckoning only)
        else:
            self.uPose = self.uHat
            self.Sigma_cov = self.Sigma_hat
            if (self.mcl_aid):
                self.tf_mutex = False

        # self.get_logger().info(f'Pose actual: x={self.uPose[0,0]:.2f}, y={self.uPose[1,0]:.2f}, θ={self.uPose[2,0]:.2f}')
        
        self.tf_mutex_pub.publish(Bool(data=self.tf_mutex))
        self.set_previous()
        elapsed_time = time.perf_counter() - start_time
        self.get_logger().info(f"[⏱️] Tiempo del ciclo Kalman: {elapsed_time:.4f} segundos")

        if (self.broadcast_tf):
            if (self.mcl_aid and not self.tf_mutex):
               return 
            self.broadcast_transform()



    def timer_callback(self):
        if(self.new_odom):
            self.new_odom = False
            self.Kalmann_filter()


def main(args=None):
    rclpy.init(args=args)
    node = KalmanNode()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
