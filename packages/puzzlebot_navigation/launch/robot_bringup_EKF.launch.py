import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    pkg_lidar_name = "rplidar_ros"
    pkg_kinematics_name = "puzzlebot_kinematics"
    pkg_vision_name = "puzzlebot_vision"

    # packages share paths
    pkg_kinematics_share = FindPackageShare(pkg_kinematics_name).find(pkg_kinematics_name)
    pkg_lidar_share = FindPackageShare(pkg_lidar_name).find(pkg_lidar_name)
    pkg_vision_share = FindPackageShare(pkg_vision_name).find(pkg_vision_name)
    
    # Launcher paths
    lidar_launch_path = os.path.join(pkg_lidar_share, 'launch', 'rplidar_a1_launch.py')
    real_kinematics_launch_path = os.path.join(pkg_kinematics_share, 'launch', 'puzzlebot_kinematic_real.launch.py')
    vision_launch_path = os.path.join(pkg_vision_share, 'launch', 'camera.launch.py')

    port_micro = LaunchConfiguration('port_micro')
    port_lidar = LaunchConfiguration('port_lidar')
    
    return LaunchDescription([
        # Launch arguments
        
        DeclareLaunchArgument(
            name="port_micro", default_value="/dev/stm32",
            description="Which USB port to use for Micro-ROS agent connection with Hackerboard"
        ),

        DeclareLaunchArgument(
            name="port_lidar", default_value="/dev/rplidar",
            description="Which USB port to use for Lidar connection"
        ),
        
        DeclareLaunchArgument(
            name="default_vision", default_value="True",
            description="Whether to activate vision submodule"
        ),

        # Lidar Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
            launch_arguments={
                'serial_port': port_lidar
            }.items()
        ),

        # Micro ROS
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "-D", port_micro],
            parameters=[],
            output='screen',
        ),

        #Camera launcher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vision_launch_path),
            condition=IfCondition(LaunchConfiguration('default_vision')),
        ),
        
        # Puzzlebot kinematics launcher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_kinematics_launch_path),
            launch_arguments={
                'rviz': 'false'
            }.items()
        ),


        #Launch map
        Node(
            package="puzzlebot_navigation",
            executable="custom_map_server.py",
            name="custom_map_server",
            output="screen",
        ),

        #Launch Aruco
        Node(
            package='puzzlebot_vision',
            executable='aruco_detector_node.py',
            name='aruco_detector_node',
            output='screen',
            parameters=[{'usingKalman': True}],
        ),

        #Launch EKF
        Node(
            package='puzzlebot_navigation',
            executable='kalmann_localization',
            name='kalmann_localization',
            output='screen'
        ),

        #Launch local map
        Node(
            package="puzzlebot_navigation",
            executable="local_map.py",
            name="local_map",
            output="screen",
            parameters=[
            ],
        ),
        
    ])
