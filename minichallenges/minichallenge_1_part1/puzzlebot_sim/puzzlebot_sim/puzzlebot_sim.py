import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np

class PuzzlebotPublisher(Node):

    def __init__(self):
        super().__init__('puzzlebot_publisher')
        self.scale = 1
        
        self.define_TF()

        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

        self.start_time = self.get_clock().now()
        self.omega = 0.1

        

    def timer_cb(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9

        self.t1.header.stamp = self.get_clock().now().to_msg()
        self.t1.transform.translation.x = 0.5*np.sin(-self.omega*elapsed_time)
        self.t1.transform.translation.y = 0.5*np.cos(self.omega*elapsed_time)
        self.t1.transform.translation.z = 0.0
        q1 = transforms3d.euler.euler2quat(0,0,self.omega*elapsed_time)
        self.t1.transform.rotation.x = q1[1]
        self.t1.transform.rotation.y = q1[2]
        self.t1.transform.rotation.z = q1[3]
        self.t1.transform.rotation.w = q1[0]

        self.t3.header.stamp = self.get_clock().now().to_msg()
        self.t3.transform.translation.x = self.scale*0.052
        self.t3.transform.translation.y = self.scale*-0.095
        self.t3.transform.translation.z = self.scale*-0.0025
        q3 = transforms3d.euler.euler2quat(-self.omega*elapsed_time, 0.0, 1.57)
        self.t3.transform.rotation.x = q3[1]
        self.t3.transform.rotation.y = q3[2]
        self.t3.transform.rotation.z = q3[3]
        self.t3.transform.rotation.w = q3[0]

        self.t4.header.stamp = self.get_clock().now().to_msg()
        self.t4.transform.translation.x = self.scale*0.052
        self.t4.transform.translation.y = self.scale*0.095
        self.t4.transform.translation.z = self.scale*-0.0025
        q4 = transforms3d.euler.euler2quat(-self.omega*elapsed_time, 0.0, 1.57)
        self.t4.transform.rotation.x = q4[1]
        self.t4.transform.rotation.y = q4[2]
        self.t4.transform.rotation.z = q4[3]
        self.t4.transform.rotation.w = q4[0]

        self.static_br1.sendTransform(self.t)
        self.tf_br2.sendTransform(self.t1)
        self.static_br3.sendTransform(self.t2)
        self.tf_br4.sendTransform(self.t3)
        self.tf_br5.sendTransform(self.t4)
        self.static_br6.sendTransform(self.t5)


    def define_TF(self):
        self.static_br1 = StaticTransformBroadcaster(self)
        self.tf_br2 = TransformBroadcaster(self)
        self.static_br3 = StaticTransformBroadcaster(self)
        self.tf_br4 = TransformBroadcaster(self)
        self.tf_br5 = TransformBroadcaster(self)
        self.static_br6 = StaticTransformBroadcaster(self)

        # Transform map - odom
        self.t = TransformStamped()
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'map'
        self.t.child_frame_id = 'odom'
        self.t.transform.translation.x = 0.5
        self.t.transform.translation.y = 0.5
        self.t.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0,0,0)
        self.t.transform.rotation.x = q[1]
        self.t.transform.rotation.y = q[2]
        self.t.transform.rotation.z = q[3]
        self.t.transform.rotation.w = q[0]

        # Transform odom - base_footprint
        self.t1 = TransformStamped()
        
        self.t1.header.frame_id = 'odom'
        self.t1.child_frame_id = 'base_footprint'
        

        # Transform base_footprint - base_link
        self.t2 = TransformStamped()
        self.t2.header.stamp = self.get_clock().now().to_msg()
        self.t2.header.frame_id = 'base_footprint'
        self.t2.child_frame_id = 'base_link'
        self.t2.transform.translation.x = self.scale*0.0
        self.t2.transform.translation.y = self.scale*0.0
        self.t2.transform.translation.z = self.scale*0.05
        q2 = transforms3d.euler.euler2quat(0.0, 0.0, 0.0)
        self.t2.transform.rotation.x = q2[1]
        self.t2.transform.rotation.y = q2[2]
        self.t2.transform.rotation.z = q2[3]
        self.t2.transform.rotation.w = q2[0]

        self.t3 = TransformStamped()

        self.t3.header.frame_id = 'base_link'
        self.t3.child_frame_id = 'wheel_r_link'
        

        self.t4 = TransformStamped()
        
        self.t4.header.frame_id = 'base_link'
        self.t4.child_frame_id = 'wheel_l_link'
        

        self.t5 = TransformStamped()
        self.t5.header.stamp = self.get_clock().now().to_msg()
        self.t5.header.frame_id = 'base_link'
        self.t5.child_frame_id = 'caster_link'
        self.t5.transform.translation.x = self.scale*-0.095
        self.t5.transform.translation.y = self.scale*0.0
        self.t5.transform.translation.z = self.scale*-0.03
        q5 = transforms3d.euler.euler2quat(0.0, 0.0, 0.0)
        self.t5.transform.rotation.x = q5[1]
        self.t5.transform.rotation.y = q5[2]
        self.t5.transform.rotation.z = q5[3]
        self.t5.transform.rotation.w = q5[0]

        

def main(args=None):
    rclpy.init(args=args)

    node = PuzzlebotPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()