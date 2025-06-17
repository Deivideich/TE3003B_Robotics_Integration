import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_urdf_name = "puzzlebot_description"
    pkg_kinematics_name = "puzzlebot_kinematics"
    # Paths
    pkg_kinematics_share = FindPackageShare(pkg_kinematics_name).find(pkg_kinematics_name)
    pkg_urdf_share = FindPackageShare(pkg_urdf_name).find(pkg_urdf_name)
    default_model_path = os.path.join(pkg_urdf_share, "urdf", "robot.xacro")
    default_rviz_config_path = os.path.join(pkg_kinematics_share, "rviz", "nav2.rviz")
    nav_launch_path = os.path.join(pkg_kinematics_share, "launch", "puzzlebot_nav_launch.py")
    gazebo_spawner_launch_path = os.path.join(pkg_kinematics_share, "launch", "puzzlebot_gazebo_spawner.launch.py")
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            name="model", default_value=default_model_path,
            description="Absolute path to robot urdf.xacro file"
        ),
        DeclareLaunchArgument(
            name="rviz_nav", default_value="true",
            description="Launch RViz?"
        ),
        
        DeclareLaunchArgument(
            name="rviz_tf", default_value="false",
            description="Launch RViz with TF?"
        ),
            
        DeclareLaunchArgument(
            name="nav", default_value="false",
            description="Launch Navigation2 stack?"
        ),
        DeclareLaunchArgument(
            name="mapping", default_value="false",
            description="Launch Mapping process?"
        ),
        DeclareLaunchArgument(
            name="gazebo_model_file", default_value=os.path.join(pkg_urdf_share, "models", "mcl_world", "model.sdf"),
            description="Path to the Gazebo model file"
        ),
        DeclareLaunchArgument(
            name="spawn_entity_name", default_value="puzzlebot",
            description="Name for the entity in Gazebo"
        ),

        # Include the Gazebo spawner launch file unconditionally with arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_spawner_launch_path),
            launch_arguments={
                "model": LaunchConfiguration("model"),
                "rviz": LaunchConfiguration("rviz_tf"),
                "prefix": "",
                "use_gazebo_controllers": "true",
                "gazebo_model_file": LaunchConfiguration("gazebo_model_file"),
                "spawn_entity_name": LaunchConfiguration("spawn_entity_name"),
            }.items()
        ),

        # Optional RViz launch
        Node(
            condition=IfCondition(LaunchConfiguration("rviz_nav")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", default_rviz_config_path]
        ),
        
        # Include Mapping Launch (slam_toolbox)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py"
                ])
            ]),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": os.path.join(
                    pkg_kinematics_share,
                    "config",
                    "mapping_params.yaml"
                )
            }.items()
        ),
        
        # Include Navigation Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_path),
            condition=IfCondition(LaunchConfiguration("nav")),
            launch_arguments={
                "use_sim_time": "true",
                "autostart": "true",
                "params_file": os.path.join(pkg_kinematics_share, "config", "nav_params.yaml")
            }.items()
        ),
    ])
