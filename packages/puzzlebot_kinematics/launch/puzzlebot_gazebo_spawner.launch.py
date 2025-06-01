import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    pkg_urdf_name = "puzzlebot_description"
    pkg_kinematics_name = "puzzlebot_kinematics"
    # Paths
    pkg__kinematics_share = FindPackageShare(pkg_kinematics_name).find(pkg_kinematics_name)
    pkg_urdf_share = FindPackageShare(pkg_urdf_name).find(pkg_urdf_name)
    default_model_path = os.path.join(pkg_urdf_share, "urdf", "robot.xacro")
    default_rviz_config_path = os.path.join(pkg__kinematics_share, "rviz", "visualizer.rviz")
    aruco_marker_path = os.path.join(pkg_urdf_share, "urdf", "aruco_marker.xacro")
    
    # Add the path for the Gazebo model (adjust based on where the saved model files are)
    gazebo_model_path = os.path.join(pkg_urdf_share, "models", "mcl_world")
    small_gazebo_model_path = os.path.join(pkg_urdf_share, "models", "PUZZLEBOT_ARENA_WALLS")
    obstacles_model_path = os.path.join(pkg_urdf_share, "urdf", "boxes.xacro")

       # === Step 2: Process the xacro file into URDF ===
    doc = xacro.process_file(obstacles_model_path)
    box_description = doc.toxml()
    
    urdf_file = '/tmp/box.urdf'
    with open(urdf_file, 'w') as f:
        f.write(doc.toxml())

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
            name="use_gazebo_controllers", default_value="true",
            description="Whether to include Gazebo controllers"
        ),
        DeclareLaunchArgument(
            name="use_gazebo_odom", default_value="true",
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
        
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen"
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
                "-file", os.path.join(small_gazebo_model_path, "model.sdf"),  # Replace with model.sdf path
                "-entity", "wall_model",  # Correct entity name here
                "-robot_namespace", "wall"
            ],
            output="screen"
        ),
        
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-file", aruco_marker_path,
                "-entity", "aruco_marker",
                "-robot_namespace", "aruco_marker",
                "-x", "1.0",  # X position
                "-y", "1.0",  # Y position
                "-z", "0.2",  # Z position
                "-R", "0",    # Roll
                "-P", "0",    # Pitch
                "-Y", "0"     # Yaw
            ],
            output="screen"
        
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-file", urdf_file,
                        "-entity", "box",
                        "-x", "1.1",
                        "-y", "0.7",
                        "-z", "0.0",
                        "-R", "0",
                        "-P", "0",
                        "-Y", "0"
                    ],
                    output="screen"
                ),
            ]
        ),
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-file", urdf_file,
                        "-entity", "box_2",
                        "-x", "0.5",
                        "-y", "1.2",
                        "-z", "0.0",
                        "-R", "0.0",
                        "-P", "0",
                        "-Y", "0"
                    ],
                    output="screen"
                ),
            ]
        ),
                
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-file", urdf_file,
                        "-entity", "box_3",
                        "-x", "1.2",
                        "-y", "0.2",
                        "-z", "0.0",
                        "-R", "0",
                        "-P", "0",
                        "-Y", "0"
                    ],
                    output="screen"
                ),
            ]
        ),
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-file", urdf_file,
                        "-entity", "box_4",
                        "-x", "0.5",
                        "-y", "0.5",
                        "-z", "0.0",
                        "-R", "0.0",
                        "-P", "0",
                        "-Y", "1.57"
                    ],
                    output="screen"
                ),
            ]
        ),

        # Optional RViz launch
        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", default_rviz_config_path]
        )
    ])