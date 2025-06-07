#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
               
    camera = Node(
        package='puzzlebot_navigation',
        executable='initial_pose_handler.py',
        name='initial_pose_handler',
        arguments=[],
        parameters=[
        ],
        output='screen'
    )
        
    camera_info = Node(
        package='puzzlebot_vision',
        executable='aruco_tf_saver.py',
        name='aruco_tf_saver',
        arguments=[],
        parameters=[
        ],
        output='screen'
    )
        
    ld = [camera, camera_info]

    return LaunchDescription(ld)
