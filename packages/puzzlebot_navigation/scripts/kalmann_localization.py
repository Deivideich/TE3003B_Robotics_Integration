import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
import math
import numpy as np

class KalmannNode(Node):
    def __init__(self):
        super().__init__('kalmann_node')

        # SUBSCRIBERS
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.wheel_radius = 0.05
        self.wheel_base = 0.19
        self.dt = 0.0

        self.omega_l = 0.0
        self.omega_r = 0.0

        self.uHat = np.zeros((1, 3))

        self.gradient_H = np.zeros((3,3))
        self.gradient_H[0,0] = 1
        self.gradient_H[1,1] = 1
        self.gradient_H[2,2] = 1

        self.Sigma_cov = np.zeros((3,3))
        self.Sigma_hat = np.zeros((3,3))
        self.error_Q = np.zeros((3,3))
        self.zHat = np.zeros((1, 2))

        self.gradient_G = np.zeros((2,3))
        self.gradient_G[1, 2] = -1

        self.Z_mat = np.zeros((2,2))
        self.R_error = np.array([[0.1, 0],
                                 [0, 0.02]])
        
        self.Kalmann_gain = np.zeros((3, 2)) #CREO
        self.uPose = np.zeros((1, 3))

    def joint_state_callback(self, msg):
        # Extract wheel velocities from JointState message
        if len(msg.velocity) < 2:
            self.get_logger().warn("Received less than 2 wheel velocities!")
            return

        omega_l = msg.velocity[0]
        omega_r = msg.velocity[1]

        # Update dt
        current_time = self.get_clock().now().seconds_nanoseconds()
        now = current_time[0] + current_time[1] * 1e-9
        self.dt = now - self.last_time
        self.last_time = now

        # Direct kinematics
        v = self.wheel_radius * (omega_r + omega_l) / 2
        w = self.wheel_radius * (omega_r - omega_l) / self.wheel_base

        

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        # Convert yaw to quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        # Create Odometry message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
        # self.get_logger().info(f'Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')

        


    def calcMiuHat(self):
        self.uHat[0] += self.dt * self.v * math.cos(self.theta)
        self.uHat[1] += self.dt * self.v * math.sin(self.theta) 
        self.uHat[2] += self.dt * self.w

    def calc_Gradient_h(self):
        self.gradient_H[0, 2] = -self.dt * self.v * math.sin(self.theta)
        self.gradient_H[1, 2] = self.dt * self.v * math.cos(self.theta)
        
    
    def calc_SigmaHat(self):
        self.Sigma_hat = self.gradient_H @ self.Sigma_cov @ self.gradient_H.T + self.error_Q
        
    def Calc_zHat(self):
        # Nos falta la posicion de los landmarks en el mapa real.
        # Dependiendo del landmark que veamos
        diff_x = self.m_x - self.uHat[0]
        diff_y = self.m_y - self.uHat[1]

        self.zHat[0] = np.sqrt( (diff_x)**2 + (diff_y)**2 )
        self.zHat[1] = math.atan2(diff_y, diff_x) - self.uHat[2]


        noise = np.random.normal(0, 0.1, size=self.zHat.shape)
        self.zHat += noise 

    def calc_Gradient_g(self):
        x = self.uHat[0]
        y = self.uHat[1]
        theta = self.uHat[2]

        #TODO

        diff_x = self.m_x - x
        diff_y = self.m_y - y
        
        square_root = np.sqrt((diff_x)**2 + (diff_y)**2)
        self.gradient_G[0, 0] = -x / square_root
        self.gradient_G[0, 1] = -y / square_root

        sum_sq = diff_y ** 2 + diff_x ** 2
        self.gradient_G[1, 0] = diff_y / sum_sq
        self.gradient_G[1, 1] = -diff_x / sum_sq

    def Calc_Z(self):
        self.Z_mat = self.gradient_G @ self.Sigma_hat @ self.gradient_G.T + self.R_error
        
    def calc_KalmannGain(self):
        self.Kalmann_gain = self.Sigma_hat @ self.gradient_G.T @ np.linalg.inv(self.Z_mat)
        

    def calc_miu(self):
        #z_vec = 
        self.uPose = self.uHat + self.Kalmann_gain @ ( - self.zHat)

    def calc_sigma(self):
        self.Sigma_cov = (np.ones((3,3)) - self.Kalmann_gain @ self.gradient_G) @ self.Sigma_hat

def main(args=None):
    rclpy.init(args=args)
    node = KalmannNode()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
