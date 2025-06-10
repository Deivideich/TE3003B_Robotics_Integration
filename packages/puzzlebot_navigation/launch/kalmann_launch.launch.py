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
    pkg_urdf_name = "puzzlebot_description"
    pkg_kinematics_name = "puzzlebot_kinematics"
    pkg_navigation_name = "puzzlebot_navigation"
    # Paths
    pkg__kinematics_share = FindPackageShare(pkg_kinematics_name).find(pkg_kinematics_name)
    pkg_nav_share = FindPackageShare(pkg_navigation_name).find(pkg_navigation_name)

    pkg_urdf_share = FindPackageShare(pkg_urdf_name).find(pkg_urdf_name)
    default_model_path = os.path.join(pkg_urdf_share, "urdf", "robot.xacro")
    default_rviz_config_path = os.path.join(pkg_nav_share, "rviz", "mcl.rviz")

    # Add the path for the Gazebo model (adjust based on where the saved model files are)
    gazebo_model_path = os.path.join(pkg_urdf_share, "models", "mcl_world")

    return LaunchDescription([
        # Launch arguments
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
            name="use_gazebo_controllers", default_value="false",
            description="Whether to include Gazebo controllers"
        ),
        DeclareLaunchArgument(
            name="use_gazebo_odom", default_value="false",
            description="Whether to include Gazebo odometry"
        ),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("gazebo_ros"),
                    "launch", "gazebo.launch.py"
                ])
            ])
        ),
    
        # State publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(
                    Command([
                        FindExecutable(name="xacro"), " ",
                        LaunchConfiguration("model"), " ",
                        "prefix:=", LaunchConfiguration("prefix"), " ",
                        "use_gazebo_controllers:=", LaunchConfiguration("use_gazebo_controllers"),
                        " ",
                        "use_gazebo_odom:=", LaunchConfiguration("use_gazebo_odom"),
                        " ",
                    ]),
                    value_type=str
                )
            }],
            output="screen"
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-topic", "/robot_description",
                        "-entity", "puzzlebot",
                        "-x", "0.0",  # X position
                        "-y", "0.0",  # Y position
                        "-z", "0.2",  # Z position
                        "-R", "0",    # Roll
                        "-P", "0",    # Pitch
                        "-Y", "0"     # Yaw
                    ],
                    output="screen"
                )
            ]
        ),

        # Spawn Gazebo model (wall model)
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",  # Using spawn_entity.py instead of spawn_model.py
            arguments=[
                "-file", os.path.join(gazebo_model_path, "model.sdf"),  # Replace with model.sdf path
                "-entity", "wall_model",  # Correct entity name here
                "-robot_namespace", "wall"
            ],
            output="screen"
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
        ),
        
        # Node(
        #     package="puzzlebot_navigation",
        #     executable="monte_carlo_localisation.py",
        #     name="monte_carlo_localisation",
        #     output="screen",
        #     parameters=[
        #         {key: LaunchConfiguration(key) for key in mcl_args.keys()}
        #     ],
        # ),


        #Node Kalmann
        


        Node(
            package="puzzlebot_navigation",
            executable="local_map.py",
            name="local_map",
            output="screen",
            parameters=[
            ],
        ),
        
    ])
