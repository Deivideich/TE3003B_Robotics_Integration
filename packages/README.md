# Puzzlebot Description Package

This package contains the robot description files (`.urdf` and `.xacro`) for the **Puzzlebot**, developed by Manchester Robotics. It supports simulation of robot kinematics, integration with Gazebo, and SLAM/Navigation using Nav2.

---

## 🧠 Kinematics Simulation

The kinematics simulation computes wheel joint states and odometry using direct and inverse kinematic equations.

To run the simulation:

```bash
ros2 launch puzzlebot_description puzzlebot_kinematic_sim.launch.py
```
## 🛠️ Gazebo Simulation

To spawn the Puzzlebot in a Gazebo environment (without mapping or navigation nodes):

```bash
ros2 launch puzzlebot_description puzzlebot_gazebo_spawner.launch.py use_gazebo_controllers:=true
```

## 🗺️ SLAM (Mapping)

To launch SLAM with `slam_toolbox` and begin building a map of the environment:

```bash
ros2 launch puzzlebot_description puzzlebot_nav_mapping.launch.py nav:=true
```

## 🤖 Navigation (Nav2)
To activate the full navigation stack using Nav2 (planner, controller, recovery behaviors):

```bash
ros2 launch puzzlebot_description puzzlebot_nav_mapping.launch.py nav:=true
```
## 🔧 Customization

`URDF Model`: Located in urdf/robot.xacro

`Gazebo Model`: Custom worlds and SDF files are under models/

`RViz Configs`: Default RViz configuration is in rviz/

## 📁 Package Structure Overview
```bash
puzzlebot_description/
├── launch/
│   ├── puzzlebot_kinematic_sim.launch.py
│   ├── puzzlebot_gazebo_spawner.launch.py
│   └── puzzlebot_nav_mapping.launch.py
├── urdf/
│   └── robot.xacro
├── models/
├── rviz/
│   └── nav2.rviz
└── config/
    ├── mapping_params.yaml
    └── nav_params.yaml
```

## Requirements

### OMPL
- Install the OMPL library for advanced motion planning capabilities.
```bash
sudo apt install ros-humble-ompl libompl-dev ompl-demos
```