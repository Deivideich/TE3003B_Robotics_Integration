#!/usr/bin/env python3

# Author: Addison Sears-Collins
# Date: December 17, 2020
# ROS Version: ROS 2 Foxy Fitzroy

import math 
import rclpy 
from time import sleep 
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import Twist     
from sensor_msgs.msg import LaserScan    
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data 

import tf_transformations
import tf2_geometry_msgs
import tf2_ros
import numpy as np 

class BugController(Node):
    
    def __init__(self):
        super().__init__('BugController')
        
        # param if sim
        self.declare_parameter('sim', False)
        self.sim = self.get_parameter('sim').get_parameter_value().bool_value
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.create_subscription(PoseWithCovarianceStamped, "/mcl_pose", self.robot_pose_callback, qos)
        
        self.subscription_goal_pose = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_received,
            qos)
        
        self.publisher_ = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10)
        
        # Initialize the LaserScan sensor readings to some large value
        # Values are in meters.
        self.left_dist = 999999.9 # Left
        self.leftfront_dist = 999999.9 # Left-front
        self.front_dist = 999999.9 # Front
        self.rightfront_dist = 999999.9 # Right-front
        self.right_dist = 999999.9 # Right

        ################### ROBOT CONTROL PARAMETERS ##################
        
        # Maximum forward speed of the robot in meters per second
        self.forward_speed = 0.08 
        
        # Current position and orientation of the robot in the global 
        # reference frame
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # By changing the value of self.robot_mode, you can alter what
        # the robot will do when the program is launched.
        #   "obstacle avoidance mode": Robot will avoid obstacles
        #   "go to goal mode": Robot will head to an x,y coordinate   
        #   "wall following mode": Robot will follow a wall 
        self.robot_mode = "go to goal mode"
        
        # Maximum left-turning speed    
        self.turning_speed = 0.25 # rad/s

        ############# GO TO GOAL MODE PARAMETERS ######################
        # Finite states for the go to goal mode
        #   "adjust heading": Orient towards a goal x, y coordinate
        #   "go straight": Go straight towards goal x, y coordinate
        #   "goal achieved": Reached goal x, y coordinate
        self.go_to_goal_state = "adjust heading"
        
        # List the goal destinations
        # We create a list of the (x,y) coordinate goals
        self.goal_x_coordinates = False # [ 0.0, 3.0, 0.0, -1.5, -1.5,  4.5, 0.0]
        self.goal_y_coordinates = False # [-4.0, 1.0, 1.5,  1.0, -3.0, -4.0, 0.0]
        
        # Keep track of which goal we're headed towards
        self.goal_idx = 0
        
        # Keep track of when we've reached the end of the goal list
        self.goal_max_idx =  None # len(self.goal_x_coordinates) - 1 
        
        # +/- 2.0 degrees of precision
        self.yaw_precision = 2.0 * (math.pi / 180) 
        
        # How quickly we need to turn when we need to make a heading
        # adjustment (rad/s)
        self.turning_speed_yaw_adjustment = 0.25
        
        # Need to get within +/- 0.2 meter (20 cm) of (x,y) goal
        self.dist_precision = 0.2

        ############# WALL FOLLOWING MODE PARAMETERS ##################     
        # Finite states for the wall following mode
        #   "turn left": Robot turns towards the left
        #   "search for wall": Robot tries to locate the wall       
        #   "follow wall": Robot moves parallel to the wall
        self.wall_following_state = "turn left"
        
        # Set turning speeds (to the left) in rad/s 
        self.turning_speed_wf_fast = 0.3  # Fast turn
        self.turning_speed_wf_slow = 0.2 # Slow turn
        
        # Wall following distance threshold.
        # We want to try to keep within this distance from the wall.
        self.dist_thresh_wf = 0.4 # in meters  
        
        # We don't want to get too close to the wall though.
        self.dist_too_close_to_wall = 0.25 # in meters
        
        self.bug0_switch = "ON"
        
        # Start-Goal Line Calculated?
        self.start_goal_line_calculated = False
        
        # Start-Goal Line Parameters
        self.start_goal_line_slope_m = 0
        self.start_goal_line_y_intercept = 0
        self.start_goal_line_xstart = 0
        self.start_goal_line_xgoal = 0
        self.start_goal_line_ystart = 0
        self.start_goal_line_ygoal = 0
        
        # Anything less than this distance means we have encountered
        # a wall. Value determined through trial and error.
        self.dist_thresh_bug0 = 0.35
        
        # Leave point must be within +/- 0.1m of the start-goal line
        # in order to go from wall following mode to go to goal mode
        self.distance_to_start_goal_line_precision = 0.15
        
        # Used to record the (x,y) coordinate where the robot hit
        # a wall.
        self.hit_point_x = 0
        self.hit_point_y = 0
        
        # Distance between the hit point and the goal in meters
        self.distance_to_goal_from_hit_point = 0.0
        
        # Used to record the (x,y) coordinate where the robot left
        # a wall.       
        self.leave_point_x = 0
        self.leave_point_y = 0
        
        # Distance between the leave point and the goal in meters
        self.distance_to_goal_from_leave_point = 0.0
        
        # The hit point and leave point must be far enough 
        # apart to change state from wall following to go to goal
        # This value helps prevent the robot from getting stuck and
        # rotating in endless circles.
        # This distance was determined through trial and error.
        self.leave_point_to_hit_point_diff = 0.25 # in meters
        
        # the range of the scanner to assume way to goal is free
        self.range_scanner_free = 90 # in degrees
        
        self.get_logger().info('BugController node has been started.')
        
    def pose_received(self,msg):
        """
        Populate the pose.
        """
        # transform to map
        target_frame = 'map'
        # transform the pose to map
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, 
                msg.header.frame_id, 
                rclpy.time.Time())
            msg.pose = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Error transforming pose: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            return
        
        # get the pose
        pose = Pose()
        pose.position.x = msg.pose.position.x
        pose.position.y = msg.pose.position.y
        pose.position.z = msg.pose.position.z
        pose.orientation.x = msg.pose.orientation.x
        pose.orientation.y = msg.pose.orientation.y
        pose.orientation.z = msg.pose.orientation.z
        pose.orientation.w = msg.pose.orientation.w
        # get the goal coordinates
        self.goal_x_coordinates = [pose.position.x]
        self.goal_y_coordinates = [pose.position.y]
        # Set the goal index to 0
        self.goal_idx = 0
        
        self.go_to_goal_state = "adjust heading"
        
        
    def scan_callback(self, msg):
        """
        This method gets called every time a LaserScan message is 
        received on the /scan ROS topic   
        """
        # (e.g. -90 degrees to 90 degrees....0 to 180 degrees)
        
        range = 2
        
        # if laser scan msg.ranges is != 360, then sample it
        if len(msg.ranges) % 360 == 0:
            # check the step needed for the sampling
            step = int(len(msg.ranges) / 360)
            msg.ranges = msg.ranges[::step]
            
        
        # clean infs
        
        if self.sim:
            self.right_dist = np.mean(msg.ranges[(90-range):(90+range)]) # Left
            self.rightfront_dist = np.mean(msg.ranges[(135-range):(135+range)])
            self.rightback_dist = np.mean(msg.ranges[(45-range):(45+range)]) # Right
            self.front_dist = np.mean(msg.ranges[(180-range):(180+range)]) # Front
            self.leftfront_dist = np.mean(msg.ranges[(225-range):(225+range)])
            self.left_dist = np.mean(msg.ranges[(270-range):(270+range)])
            self.leftback_dist = np.mean(msg.ranges[(315-range):(315+range)]) # Left-back
        else:
            self.front_dist = np.mean(np.concatenate([
                msg.ranges[360-range:],
                msg.ranges[:range]]))
            self.leftfront_dist = np.mean(msg.ranges[(45-range):(45+range)]) # Left-front
            self.left_dist = np.mean(msg.ranges[(90-range):(90+range)]) # Left
            self.leftback_dist = np.mean(msg.ranges[(135-range):(135+range)]) # Left-back
            self.rightfront_dist = np.mean(msg.ranges[(315-range):(315+range)]) # Right-front
            self.right_dist = np.mean(msg.ranges[(270-range):(270+range)]) # Right
            self.rightback_dist = np.mean(msg.ranges[(225-range):(225+range)]) # Right-back
        
        # Print the distance values (in meters) for testing
        self.get_logger().info('L:%f LF:%f F:%f RF:%f R:%f' % (
            self.left_dist,
            self.leftfront_dist,
            self.front_dist,
            self.rightfront_dist,
            self.right_dist))
        
        self.curr_scan = msg
        
        if self.goal_x_coordinates == False and self.goal_y_coordinates == False:
            return
        
            
    def robot_pose_callback(self, msg):
        """
        Extract the position and orientation data. 
        This callback is called each time
        a new message is received on the '/en613/state_est' topic
        """
        # Update the current estimated state in the global reference frame
        pose = msg
        pos = pose.pose.pose.position
        ori = pose.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.current_x = pos.x
        self.current_y = pos.y
        self.current_yaw = yaw
        
        # Wait until we have received some goal destinations.
        if self.goal_x_coordinates == False and self.goal_y_coordinates == False:
            return
        
        # See if the bug0 algorithm is activated. If yes, call bug0()
        if self.bug0_switch == "ON":
            self.bug0()
        else:
            
            if self.robot_mode == "go to goal mode":
                self.go_to_goal()
            elif self.robot_mode == "wall following mode":
                self.follow_wall()
            else:
                pass 
                            
    def go_to_goal(self):
        """
        This code drives the robot towards to the goal destination
        """
        # Create a geometry_msgs/Twist message
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        # If bug0 algorithm is activated
        if self.bug0_switch == "ON":
        
            # If the wall is in the way
            d = self.dist_thresh_bug0
            if (    self.leftfront_dist < d or
                self.front_dist < d or
                self.rightfront_dist < d):
            
                # Change the mode to wall following mode.
                self.robot_mode = "wall following mode"
            
        # Fix the heading       
        if (self.go_to_goal_state == "adjust heading"):
            
            desired_yaw = math.atan2(
                    self.goal_y_coordinates[self.goal_idx] - self.current_y,
                    self.goal_x_coordinates[self.goal_idx] - self.current_x)
            
            yaw_error = desired_yaw - self.current_yaw
            
            # Normalize yaw_error to [-pi, pi] to ensure shortest angular path
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi
            
            if math.fabs(yaw_error) > self.yaw_precision:
            
                if yaw_error > 0:    
                    # Turn left (counterclockwise)      
                    msg.angular.z = self.turning_speed_yaw_adjustment               
                else:
                    # Turn right (clockwise)
                    msg.angular.z = -self.turning_speed_yaw_adjustment
                
                # Command the robot to adjust the heading
                self.publisher_.publish(msg)
                
            # Change the state if the heading is good enough
            else:               
                # Change the state
                self.go_to_goal_state = "go straight"
                
                # Command the robot to stop turning
                self.publisher_.publish(msg)        

        # Go straight                                       
        elif (self.go_to_goal_state == "go straight"):
            
            position_error = math.sqrt(
                        pow(
                        self.goal_x_coordinates[self.goal_idx] - self.current_x, 2)
                        + pow(
                        self.goal_y_coordinates[self.goal_idx] - self.current_y, 2)) 
                        
            
            # If we are still too far away from the goal                        
            if position_error > self.dist_precision:

                # Move straight ahead
                msg.linear.x = self.forward_speed * 1.5
                    
                # Command the robot to move
                self.publisher_.publish(msg)
            
                # Check our heading         
                desired_yaw = math.atan2(
                    self.goal_y_coordinates[self.goal_idx] - self.current_y,
                    self.goal_x_coordinates[self.goal_idx] - self.current_x)
                
                # How far off is the heading?   
                yaw_error = desired_yaw - self.current_yaw      
        
                # Check the heading and change the state if there is too much heading error
                if math.fabs(yaw_error) > self.yaw_precision:
                    
                    # Change the state
                    self.go_to_goal_state = "adjust heading"
                
            # We reached our goal. Change the state.
            else:           
                # Change the state
                self.go_to_goal_state = "goal achieved"
                self.get_logger().info(f"Goal {self.goal_idx} achieved!")
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0
                # Command the robot to stop
                self.publisher_.publish(msg)
        
        else:
            pass
    
    def is_way_to_goal_free(self):
        # Check if the way to the goal is free
        """
        This method checks if the way to the goal is free by checking
        the laser scan readings within a certain range.
        Returns True if the way is free, False otherwise.
        """
        # Get the angle range for checking the laser scan readings
        angle_range = self.range_scanner_free
        angle_to_goal = math.atan2(
            self.goal_y_coordinates[self.goal_idx] - self.current_y,
            self.goal_x_coordinates[self.goal_idx] - self.current_x)
        # to robot frame
        angle_to_goal = angle_to_goal - self.current_yaw
        # Normalize the angle to be within 0 to 2*pi
        angle_to_goal = angle_to_goal % (2 * np.pi)
        # Calculate the start and end angles for the laser scan readings
        # to degrees
        angle_to_goal_deg = np.rad2deg(angle_to_goal)
        start_angle = angle_to_goal_deg - angle_range / 2
        end_angle = angle_to_goal_deg + angle_range / 2
        # consider 180 is the front of the robot
        if self.sim:
            start_index = int((start_angle + 180))
            end_index = int((end_angle + 180))
        else:
            start_index = int(start_angle)
            end_index = int(end_angle)
        # ensure between 0 and 360 degrees
        start_index = start_index % 360
        end_index = end_index % 360
        # Check the laser scan readings within the specified range
        
        # if the start index is greater than the end index, it means we need to wrap around
        if start_index > end_index:
            # Check the readings from start_index to 359 and from 0 to end_index
            for i in range(start_index, 360):
                if self.curr_scan.ranges[i] < self.dist_thresh_bug0:
                    return False
            for i in range(0, end_index + 1):
                if self.curr_scan.ranges[i] < self.dist_thresh_bug0:
                    return False
        else:
            for i in range(start_index, end_index + 1):
                # If any reading is less than the distance threshold, return False
                if self.curr_scan.ranges[i] < self.dist_thresh_bug0:
                    return False
                
        return True
        
    
    def follow_wall(self):
        """
        This method causes the robot to follow the boundary of a wall.
        """
        # Create a geometry_msgs/Twist message
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0        

        if self.bug0_switch == "ON":
            
            # bug0
            way_to_goal_free = self.is_way_to_goal_free()
            if way_to_goal_free:
                # If no OBSTACLE seen, immediately go for goal
                # Change the mode to go to goal
                self.robot_mode = "go to goal mode"
                self.get_logger().info(f"Way to goal free: {way_to_goal_free}")
                return
        
        # Logic for following the wall
        # >d means no wall detected by that laser beam
        # <d means an wall was detected by that laser beam
        
        d = self.dist_thresh_wf
        
        left_covered = self.left_dist < d # or self.leftback_dist < d
        leftfront_covered = self.leftfront_dist < d
        front_covered = self.front_dist < d
        rightfront_covered = self.rightfront_dist < d
        right_covered = self.right_dist < d # or self.rightback_dist < d
        
        if self.front_dist > d and self.rightfront_dist > d and not right_covered:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_fast # turn right to find wall
            
        elif (self.front_dist > d and (self.rightfront_dist < d or right_covered)):
            if (self.rightfront_dist < self.dist_too_close_to_wall or self.right_dist < self.dist_too_close_to_wall):
                # Getting too close to the wall
                self.wall_following_state = "turn left too close"
                msg.linear.x = self.forward_speed * 0.5
                msg.angular.z = self.turning_speed_wf_fast      
            else:           
                # Go straight ahead
                self.wall_following_state = "follow wall" 
                msg.linear.x = self.forward_speed
            
        else:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast
        
        self.get_logger().info(f"Wall Following State: {self.wall_following_state}")

        self.publisher_.publish(msg)    
        
    def bug0(self):
    
        if self.go_to_goal_state == "goal achieved":
            return
            
        if self.robot_mode == "go to goal mode":
            self.get_logger().info('Going to goal...')
            self.go_to_goal()           
        elif self.robot_mode == "wall following mode":
            self.follow_wall()
        
def main(args=None):

    rclpy.init(args=args)
    controller = BugController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()