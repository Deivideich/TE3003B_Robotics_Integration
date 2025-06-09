import rclpy
from rclpy.node import Node
import yaml
import time
from geometry_msgs.msg import PoseStamped, Pose
import tf2_geometry_msgs
from puzzlebot_manager.utils.decorators import mockable
from puzzlebot_interfaces.msg import QRCodeArray
from puzzlebot_interfaces.msg import ImageClassification
import math
import tf_transformations
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
import copy


class VisionManager():
    def __init__(self, node: Node, mock : bool = False):
        
        self.node = node
        self.mock_data = False
        
        self.qr_codes = []
        
        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        self.available_qr_detection = False
        self.qr_detections_sub = self.node.create_subscription(
            QRCodeArray,
            '/vision/qr_detections',
            self.qr_detection_callback,
            qos
        )
        self.available_inference = False
        self.truck_classification_sub = self.node.create_subscription(
            ImageClassification,
            '/vision/truck_classification',
            self.truck_classification_callback,
            qos
        )
        
        self.mock_data = mock
        self.node.get_logger().info("Initializing Vision Manager...")
    
    def get_qr_poses(self, fake_qr_pose, wait=False):
        self.get_qr_codes(wait=wait)
        if len(self.qr_codes) == 0:
            return []
        
        qr_codes = copy.deepcopy(self.qr_codes)
        
        self.node.get_logger().info(f"Getting poses for {qr_codes} QRs")

        # Get the robot's pose in the map frame using tf_buffer
        tf_buffer = self.node.tf_buffer
        try:
            transform = tf_buffer.lookup_transform(
                "map",  # target frame
                "base_link",  # source frame
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            base_link_pose = PoseStamped()
            base_link_pose.header.frame_id = "base_link"
            base_link_pose.header.stamp = self.node.get_clock().now().to_msg()
            base_link_pose.pose.orientation.w = 1.0  # Identity quaternion
            base_link_pose_in_map = tf2_geometry_msgs.do_transform_pose(base_link_pose.pose, transform)
        except Exception as e:
            self.node.get_logger().error(f"Failed to get base_link pose in map frame: {e}")
            return None

        
        min_distance = float('inf')
        closest_qr_code = None

        for qr_code in qr_codes:
            if (qr_code.pose_stamped.header.frame_id != "map"):
                self.node.get_logger().info(f"Detected qr pose not in map frame, real frame: {qr_code.pose_stamped.header.frame_id}")
                continue

            distance = ((qr_code.pose_stamped.pose.position.x - base_link_pose_in_map.position.x) ** 2 + 
                        (qr_code.pose_stamped.pose.position.y - base_link_pose_in_map.position.y) ** 2) ** 0.5
            
            if distance < min_distance:
                min_distance = distance
                closest_qr_code = qr_code

        if closest_qr_code is None and not fake_qr_pose:
            return None
        
        trans_offset_array = [0.25, 0.0]  # Offsets for pre_pick and pick

        goal_array = []

        

        source_frame = f'qr_code_{closest_qr_code.content}' if not fake_qr_pose else 'base_link'
        transform = tf_buffer.lookup_transform(
            "map",  # target
            source_frame,  # source
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=1.0)
        )
        
        for i in range(len(trans_offset_array)):
            goal_pose_in_map = PoseStamped()
            goal_pose_in_map.header.frame_id = "map"
            goal_pose_in_map.header.stamp = self.node.get_clock().now().to_msg()
            
            pose_qr = Pose()
            pose_qr.position.x = 0.0
            pose_qr.position.y = 0.0
            pose_qr.position.z = trans_offset_array[i]

            q = tf_transformations.quaternion_from_euler(0, math.pi/2, 0)
            pose_qr.orientation.x = q[0]
            pose_qr.orientation.y = q[1]
            pose_qr.orientation.z = q[2]
            pose_qr.orientation.w = q[3]

            self.node.get_logger().info(f"Trying to get pose position.x: {pose_qr.position.x},  position.y: {pose_qr.position.y},  position.z: {pose_qr.position.z},  orientation.x: {pose_qr.orientation.x},  orientation.y: {pose_qr.orientation.y},  orientation.z: {pose_qr.orientation.z},  orientation.w: {pose_qr.orientation.w},")

            try:
                goal_pose_in_map.pose = tf2_geometry_msgs.do_transform_pose(pose_qr, transform)
                goal_array.append(goal_pose_in_map)
            except Exception as e:
                self.node.get_logger().error(f"Failed to transform QR code pose: {e}")
                continue
            
            


        return goal_array

    def get_qr_codes(self, wait = False, timeout=3.0):
        """
        Get the list of detected QR codes.
        """
        if wait:
            self.available_qr_detection = False

        initial_time = time.time()

        while not self.available_qr_detection and time.time() - initial_time < timeout:
            time.sleep(0.01)

        return self.qr_codes
        
    def qr_detection_callback(self, msg: QRCodeArray):
        """
        Callback function to handle received QR code detections.
        """
        self.qr_codes = msg.qrcodes
        self.available_qr_detection = len(self.qr_codes) > 0
        
    def truck_classify(self, wait=False, timeout=5.0):
        """
        Request truck classification from the vision system.
        """
        if wait:
            self.available_inference = False
        
        initial_time = time.time()

        while not self.available_inference and time.time() - initial_time < timeout:
            time.sleep(0.01)
        
        return self.truck_classification if hasattr(self, "truck_classification") else "fake_truck"
        
    def truck_classification_callback(self, msg: ImageClassification):
        """
        Callback function to handle received truck classification results.
        """
        self.available_inference = True
        self.truck_classification = msg.label_name