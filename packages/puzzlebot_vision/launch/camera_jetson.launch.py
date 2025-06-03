#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
               
    camera = Node(
        package='ros_deep_learning',
        executable='video_source',
        name='video_source',
        arguments=[],
        parameters=[
        {"resource": "csi://0"},
        {"width": 200},
        {"height": 100},
        {"codec": "unknown"},
        {"loop": 0},
        {"latency": 2000}
        ],
        output='screen'
    )
        
    camera_info = Node(
        package='camera_info_publisher',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        arguments=[],
        parameters=[
                {"camera_calibration_file": "file:///home/puzzlebot/.ros/jetson_cam.yaml"},
                {"frame_id": "camera"}
        ],
        output='screen'
    )

    camera_compressed = Node(
        package='puzzlebot_vision',
        executable='camera_compressed_pub.py',
        name='camera_compressed_pub',
        parameters=[
            {"width": 200},
            {"height": 100}
        ],
        output='screen'
    )
        
    ld = [camera, camera_info, camera_compressed]

    return LaunchDescription(ld)
