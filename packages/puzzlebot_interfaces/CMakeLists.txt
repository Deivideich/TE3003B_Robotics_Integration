cmake_minimum_required(VERSION 3.8)
project(puzzlebot_interfaces)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

# Recursively find all .msg, .srv, and .action files
file(GLOB_RECURSE action_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "action/*.action")
file(GLOB_RECURSE srv_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "srv/*.srv")
file(GLOB_RECURSE msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")

# Generate interfaces for found .msg, .srv, and .action files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs # Add dependencies here
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()