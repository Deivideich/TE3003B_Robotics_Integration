import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from std_msgs.msg import Header, Int32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
import cv2
import tf_transformations
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster, TransformStamped, Buffer, TransformListener
import tf2_ros
import time


class KalmanNode(Node):
    def __init__(self):
        super().__init__('kalman_node')

        # SUBSCRIBERS
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Int32, '/aruco_id', self.aruco_callback, 10) #Subscriber del ARUCO id
        # Aqui tiene que ir el subscriber que me de la posicion de los marcadores

        #PUBLISHERS
        # self.pub_pos = self.create_publisher(PoseStamped, '/estimated_pose', 10)
        self.pub_pos = self.create_publisher(PoseWithCovarianceStamped, '/estimated_pose', 10)

        self.timer = self.create_timer(0.05, self.timer_callback, 10)

        self.wheel_radius = 0.05
        self.wheel_base = 0.19 #0.168?
        self.dt = 0.0
        self.last_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                         self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        self.omega_l = 0.0
        self.omega_r = 0.0

        self.uHat = np.zeros((3, 1))
        self.theta_prev = 0.0

        self.gradient_H = np.zeros((3,3))
        self.gradient_H[0,0] = 1
        self.gradient_H[1,1] = 1
        self.gradient_H[2,2] = 1

        self.Sigma_cov = np.zeros((3,3))
        self.Sigma_hat = np.zeros((3,3))
        
        self.error_Q = np.zeros((3,3)) # Process noise covariance matrix
        self.K_R = 0.30406416057210744
        self.K_L = 0.38899148975615183

        self.zHat = np.zeros((2, 1))

        self.valid_id = [0, 1, 2, 3, 4, 5, 6, 7] # Valid ARUCO IDs

        self.gradient_G = np.zeros((2,3))
        self.gradient_G[1, 2] = -1

        self.Z_mat = np.zeros((2,2))
        self.R_error = np.array([[0.1, 0],
                                 [0, 0.02]])
        
        self.omega_l = 0.0
        self.omega_r = 0.0
        
        self.identity = np.eye(3) # Identity matrix
        
        self.Kalmann_gain = np.zeros((3, 2)) 
        self.uPose = np.zeros((3, 1))
        self.landmark_status = False
        self.new_odom = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
        sigma_[0,0] = self.K_R * abs(self.omega_r)
        sigma_[1,1] = self.K_L * abs(self.omega_l)

        self.error_Q = delta_w @ sigma_ @ delta_w.T

    def joint_state_callback(self, msg):
        # Extract wheel velocities from JointState message
        if len(msg.velocity) < 2:
            self.get_logger().warn("Received less than 2 wheel velocities!")
            return

        
        self.omega_l = msg.velocity[0]
        self.omega_r = msg.velocity[1]

        # Update dt
        current_time = self.get_clock().now().seconds_nanoseconds()
        now = current_time[0] + current_time[1] * 1e-9
        self.dt = now - self.last_time
        self.last_time = now

        # Direct kinematics
        self.v = self.wheel_radius * (self.omega_r + self.omega_l) / 2
        self.w = self.wheel_radius * (self.omega_r - self.omega_l) / self.wheel_base

        self.new_odom = True
    
    def calcMiuHat(self):

        self.uHat[0] += self.dt * self.v * math.cos(self.theta_prev)
        self.uHat[1] += self.dt * self.v * math.sin(self.theta_prev) 
        self.uHat[2] += self.dt * self.w
        self.uHat[2] = (self.uHat[2] + math.pi) % (2 * math.pi) - math.pi #Corregir theta

    def calc_Gradient_h(self):
        self.gradient_H[0, 2] = -self.dt * self.v * math.sin(self.theta_prev)
        self.gradient_H[1, 2] = self.dt * self.v * math.cos(self.theta_prev)
        
    
    def calc_SigmaHat(self):
        self.Sigma_hat = self.gradient_H @ self.Sigma_cov @ self.gradient_H.T + self.error_Q

    def obtain_tfs(self):
        try:
            now = self.get_clock().now()
            self.aruco_tf = self.tf_buffer.lookup_transform(
                    'map',  # target frame - map
                    f'aruco_{self.marker_id}',      # source frame - aruco
                    now)  # TODO

            self.aruco_to_robot_tf = self.tf_buffer.lookup_transform(
                'base_footprint',           # target frame - base footprint
                f'aruco_{self.marker_id}',      # source frame - aruco
                now)  #TODO
            


        except Exception as e:
            self.get_logger().warn(f'Error obtaining transforms: {str(e)}')
            return False
    
        return True
            
        
    def Calc_zHat(self):
        # Nos falta la posicion de los landmarks en el mapa real.
        # Dependiendo del landmark que veamos

        self.m_x = self.aruco_tf.transform.position.x
        self.m_y = self.aruco_tf.transform.position.y

        diff_x = self.m_x - self.uHat[0]
        diff_y = self.m_y - self.uHat[1]

        self.zHat[0] = np.sqrt( (diff_x)**2 + (diff_y)**2 )
        self.zHat[1] = math.atan2(diff_y, diff_x) - self.uHat[2] #TODO
        self.zHat[1] = (self.zHat[1] + math.pi) % (2 * math.pi) - math.pi #Normalizar angulo



        # noise = np.random.normal(0, 0.1, size=self.zHat.shape)
        # self.zHat += noise 

    def calc_Gradient_g(self):
        #TODO

        diff_x = self.m_x - self.uHat[0]
        diff_y = self.m_y - self.uHat[1]
        
        sum_sq = diff_y ** 2 + diff_x ** 2
        
        self.gradient_G[0, 0] = -diff_x / np.sqrt(sum_sq)
        self.gradient_G[0, 1] = -diff_y / np.sqrt(sum_sq)
        self.gradient_G[0, 2] = 0.0

        self.gradient_G[1, 0] = diff_y / sum_sq
        self.gradient_G[1, 1] = -diff_x / sum_sq
        self.gradient_G[1, 2] = -1.0

    def Calc_Z(self):
        self.Z_mat = self.gradient_G @ self.Sigma_hat @ self.gradient_G.T + self.R_error
        
    def calc_KalmannGain(self):
        self.Kalmann_gain = self.Sigma_hat @ self.gradient_G.T @ np.linalg.inv(self.Z_mat)
        
    def euclidean_distance(self, x, y):
        return np.sqrt( x*x + y*y )
    
    def yaw_from_quaternion(self, q):
        try:
            _, _, yaw = tf_transformations.euler_from_quaternion(q)
            return yaw
        except Exception as e:
            self.get_logger().warn(f"Error converting quaternion to euler: {str(e)}")

            return 0.0

    def calc_miu(self):
        z_vec = np.zeros((2, 1)) # This needs to be the SinglePoseMarker with Transforms. I need euclidean distance and angle from base_footprint I think?

        x = self.aruco_to_robot_tf.transform.position.x
        y = self.aruco_to_robot_tf.transform.position.y

        q = self.aruco_to_robot_tf.transform.rotation
        

        z_vec[0] = self.euclidean_distance(x, y)
        z_vec[1] = self.yaw_from_quaternion(q)

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

    def set_previous(self):
        self.uHat[0] = self.uPose[0]
        self.uHat[1] = self.uPose[1]
        self.uHat[2] = self.uPose[2]
        self.theta_prev = self.uPose[2]

        q = self.euler_to_quaternion(0.0, 0.0, self.uPose[2])
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'pose_kalman' #TODO
        msg.pose.position.x = self.uPose[0]
        msg.pose.position.y = self.uPose[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

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

        self.pub_pos.publish(msg) 
        
        
    def aruco_callback(self, msg):
        self.marker_id = msg.status
        if(id in self.valid_id):
            self.landmark_status = True
        else:
            self.landmark_status = False

    def Kalmann_filter(self):

        self.obtain_Q()
        
        self.calcMiuHat()
        self.calc_Gradient_h()
        self.calc_SigmaHat()
        # Check if landmark is visible, to correct using observations
        if(self.landmark_status):
            self.obtain_tfs()
            self.Calc_zHat()
            self.calc_Gradient_h()
            self.Calc_Z()
            self.calc_KalmannGain()
            self.calc_miu()
            self.calc_sigma()

        # If landmark is not visible, use prediction (Dead Reckoning only)
        else:
            self.uPose = self.uHat
            self.Sigma_cov = self.Sigma_hat

        self.set_previous()

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
