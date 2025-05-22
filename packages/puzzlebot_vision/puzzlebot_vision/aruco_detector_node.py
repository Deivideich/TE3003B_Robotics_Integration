#!/usr/bin/env python3

import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from rclpy.node import Node
import numpy as np
import os
import importlib.resources
import json
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


from puzzlebot_vision.aruco_detector.ArucoDetector import ArucoDetector

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        self.aruco_detector = ArucoDetector()
        self.bridge = CvBridge()
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/video_source/compressed',
            self.compressed_image_callback,
            10
        )
        
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",  # or "/estimated_pose" if not using AMCL
            10
        )
        
        self.aruco_pose_publisher = self.create_publisher(
            PoseStamped,
            "/aruco_pose",
            10
        )

        
        self._load_marker_poses_and_publish_static_tfs()
    
    def get_transform_matrix(self, rvec, tvec):
        rvec = np.array(rvec, dtype=np.float64)
        tvec = np.array(tvec, dtype=np.float64)
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        return T
    
    def invert_transform(self, T):
        T_inv = np.eye(4)
        T_inv[:3, :3] = T[:3, :3].T
        T_inv[:3, 3] = -T[:3, :3].T @ T[:3, 3]
        return T_inv

    def transform_to_pose(self, T, frame_id="map", stamp=None):
        from geometry_msgs.msg import PoseStamped
        pose = PoseStamped()
        pose.header.stamp = stamp or rclpy.time.Time().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = T[0, 3]
        pose.pose.position.y = T[1, 3]
        pose.pose.position.z = T[2, 3]
        r = R.from_matrix(T[:3, :3])
        q = r.as_quat()  # returns [x, y, z, w]        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose
    
    def _load_marker_poses_and_publish_static_tfs(self):
        try:
            with importlib.resources.files('puzzlebot_vision.aruco_detector.aruco_map_poses').joinpath('aruco_poses.json').open('r') as f:
                marker_data = json.load(f)
                self.marker_poses = {}

                for entry in marker_data:
                    marker_id = entry["id"]
                    self.marker_poses[marker_id] = entry

                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = "map"
                    t.child_frame_id = f"aruco_{marker_id}"

                    t.transform.translation.x = entry["translation"]["x"]
                    t.transform.translation.y = entry["translation"]["y"]
                    t.transform.translation.z = entry["translation"]["z"]
                    t.transform.rotation.x = entry["rotation"]["x"]
                    t.transform.rotation.y = entry["rotation"]["y"]
                    t.transform.rotation.z = entry["rotation"]["z"]
                    t.transform.rotation.w = entry["rotation"]["w"]

                    self.static_tf_broadcaster.sendTransform(t)
                
                self.get_logger().info("✅ Published static map → aruco_<id> transforms.")
                return True
        except FileNotFoundError:
            self.get_logger().error("❌ Failed to load ArUco poses file.")
            self.marker_poses = {}
            return False


    def compressed_image_callback(self, msg):
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Failed to process compressed image: {e}')

    def process_frame(self, frame):
        detections = self.aruco_detector.detect(frame)

        for det in detections:
            marker_id = det['id']

            if marker_id not in self.marker_poses:
                self.get_logger().warn(f"⚠️ Marker ID {marker_id} not in map.")
                continue

            # Publish dynamic transform: aruco -> camera_link
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f"aruco_{marker_id}"  # parent
            t.child_frame_id = "camera_link"          # child

            t.transform.translation.x = det["tvec"][0]
            t.transform.translation.y = det["tvec"][1]
            t.transform.translation.z = det["tvec"][2]

            r = cv2.Rodrigues(np.array(det["rvec"]))[0]
            q = R.from_matrix(r).as_quat()  # [x, y, z, w]
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)


    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect ArUco markers
        detections = self.aruco_detector.detect(frame)

        for det in detections:
            self.get_logger().info(
                f"🟢 Marker ID {det['id']}:\n"
                f"    Translation: x={det['tvec'][0]:.2f}, y={det['tvec'][1]:.2f}, z={det['tvec'][2]:.2f}\n"
                f"    Rotation vector: {det['rvec']}"
            )
            
            marker_id = det['id']
            if marker_id not in self.marker_poses:
                self.get_logger().warn(f"⚠️ Marker ID {marker_id} not in map.")
                continue
            
            self.get_logger().info(f"🟢 Marker ID {marker_id} detected.")

            # Get map->aruco transform
            marker_data = self.marker_poses[marker_id]
            rvec_map = np.array(marker_data["rotation"])
            tvec_map = np.array(marker_data["translation"])
            T_map_to_aruco = self.get_transform_matrix(rvec_map, tvec_map)

            # Get camera->aruco and invert it
            T_camera_to_aruco = self.get_transform_matrix(det["rvec"], det["tvec"])
            T_aruco_to_camera = self.invert_transform(T_camera_to_aruco)

            # map → base_link = map → aruco × aruco → camera
            T_map_to_base = T_map_to_aruco @ T_aruco_to_camera

            # Convert to ROS pose message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.pose = self.transform_to_pose(T_map_to_base).pose
            pose_msg.pose.covariance = [0.0]*36  # Optional

            self.pose_publisher.publish(pose_msg)

    
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()