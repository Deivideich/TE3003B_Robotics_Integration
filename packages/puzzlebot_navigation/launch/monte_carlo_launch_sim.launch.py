import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

mcl_args = {
    'useClustering': False,
    'numParticles': 1000,
    'minClusterDistance': 0.5,
    'clusterEps': 0.5,
    'clusterMinSamples': 0.05,
    'scaleRdParticles': 0.0,
    'minDistance': 0.01,
    'minAngle': 5.0,
    'repropagateCountNeeded': 1,
    'HZ' : 20.0,
}

def generate_launch_description():
    pkg_urdf_name = "puzzlebot_description"
    pkg_kinematics_name = "puzzlebot_kinematics"
    pkg_navigation_name = "puzzlebot_navigation"
    # Paths
    pkg__kinematics_share = FindPackageShare(pkg_kinematics_name).find(pkg_kinematics_name)
    pkg_nav_share = FindPackageShare(pkg_navigation_name).find(pkg_navigation_name)

    pkg_urdf_share = FindPackageShare(pkg_urdf_name).find(pkg_urdf_name)
    default_model_path = os.path.join(pkg_urdf_share, "urdf", "robot.xacro")
    default_rviz_config_path = os.path.join(pkg_nav_share, "rviz", "mcl.rviz")
    gazebo_spawner_launch_path = os.path.join(pkg__kinematics_share, "launch", "puzzlebot_gazebo_spawner.launch.py")
    # Add the path for the Gazebo model (adjust based on where the saved model files are)
    gazebo_model_path = os.path.join(pkg_urdf_share, "models", "mcl_world")

    return LaunchDescription([
        # Launch arguments
        *[
            DeclareLaunchArgument(
                name=key,
                default_value=str(value) if isinstance(value, (int, float)) else ("true" if value else "false"),
                description=f"Parameter {key} for monte_carlo_localisation node"
            )
            for key, value in mcl_args.items()
        ],
        DeclareLaunchArgument(
            name="model", default_value=default_model_path,
            description="Absolute path to robot urdf.xacro file"
        ),
        DeclareLaunchArgument(
            name="rviz", default_value="false",
            description="Launch RViz?"
        ),
        DeclareLaunchArgument(
            name="prefix", default_value="",
            description="Prefix for robot link/joint names"
        ),
        DeclareLaunchArgument(
            name="use_gazebo_controllers", default_value="true",
            description="Whether to include Gazebo controllers"
        ),
        DeclareLaunchArgument(
            name="use_gazebo_odom", default_value="false",
            description="Whether to include Gazebo odometry"
        ),
         DeclareLaunchArgument(
            name="gazebo_model_file", default_value=os.path.join(pkg_urdf_share, "models", "mcl_world", "model.sdf"),
            description="Path to the Gazebo model file"
        ),
        DeclareLaunchArgument(
            name="spawn_entity_name", default_value="puzzlebot",
            description="Name for the entity in Gazebo"
        ),
        DeclareLaunchArgument(
            name="use_mcl_clustering", default_value="false",
            description="Whether to use clustering in MCL algorithm"
        ),
    
        # # State publisher
        # Node(
        #     package="robot_state_publisher",
        #     executable="robot_state_publisher",
        #     name="robot_state_publisher",
        #     parameters=[{
        #         "robot_description": ParameterValue(
        #             Command([
        #                 FindExecutable(name="xacro"), " ",
        #                 LaunchConfiguration("model"), " ",
        #                 "prefix:=", LaunchConfiguration("prefix"), " ",
        #                 "use_gazebo_controllers:=", LaunchConfiguration("use_gazebo_controllers"),
        #                 " ",
        #                 "use_gazebo_odom:=", LaunchConfiguration("use_gazebo_odom"),
        #                 " ",
        #             ]),
        #             value_type=str
        #         )
        #     }],
        #     output="screen"
        # ),

        # Include external launch file
        # Include the Gazebo spawner launch file unconditionally with arguments
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_spawner_launch_path),
            launch_arguments={
                "model": LaunchConfiguration("model"),  
                "prefix": "",
                "use_gazebo_controllers": "true",
                "gazebo_model_file": LaunchConfiguration("gazebo_model_file"),
                "spawn_entity_name": LaunchConfiguration("spawn_entity_name"),
            }.items()
        ),


        # Optional RViz launch
        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", default_rviz_config_path]
        ),
        
        # Custom puzzlebot nodes
        Node(
            package="puzzlebot_kinematics",
            executable="differential_inverse_kinematics.py",
            name="differential_inverse_kinematics",
            output="screen",
        ),

        Node(
            package="puzzlebot_kinematics",
            executable="differential_direct_kinematics.py",
            name="differential_direct_kinematics",
            output="screen",
        ),

        Node(
            package="puzzlebot_kinematics",
            executable="puzzlebot_transforms.py",
            name="puzzlebot_transforms",
            output="screen",
        ),

        Node(
            package="puzzlebot_kinematics",
            executable="wheel_transform_broadcaster.py",
            name="wheel_transform_broadcaster",
            output="screen",
        ),
        
        Node(
            package="puzzlebot_navigation",
            executable="custom_map_server.py",
            name="custom_map_server",
            output="screen",
            # parameters=[{"map_yaml_file" : LaunchConfiguration("map_file")}],
        ),
        
        Node(
            package="puzzlebot_navigation",
            executable="monte_carlo_localisation.py",
            name="monte_carlo_localisation",
            output="screen",
            parameters=[
                {key: LaunchConfiguration(key) for key in mcl_args.keys()}
            ],
        ),

        Node(
            package="puzzlebot_navigation",
            executable="local_map.py",
            name="local_map",
            output="screen",
            parameters=[
            ],
        ),
        
    ])
