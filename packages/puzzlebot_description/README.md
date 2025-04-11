# Robot description package

This packages contains the description (.urdf and .xacro) files to use the Puzzlebot from Manchester Robotics

## Gazebo simulation

In order to spawn the robot in Gazebo without any additional mapping packages, run:

```bash
ros2 launch puzzlebot_description puzzlebot_nav_mapping.launch.py
```

In order to run the slam_toolbox use:

```bash
ros2 launch puzzlebot_description puzzlebot_nav_mapping.launch.py 
```

To run the navigation package (which spawns the navigator and planner nodes from Nav2) run:

```bash
ros2 launch puzzlebot_description puzzlebot_nav_launch.py
```
