cmake_minimum_required(VERSION 3.8)
project(puzzlebot_planning)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Keep other find_package calls if needed later, but comment out for now
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)

# Add model library
add_library(puzzlebot_planning_model SHARED
    src/model/se2_state.cpp
)
target_include_directories(puzzlebot_planning_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Link model library against necessary dependencies (if any, none for now)
ament_target_dependencies(puzzlebot_planning_model PUBLIC
    # Add dependencies like rclcpp if model needs ROS types directly
)

# Add the ROS test node executable
add_executable(test_state_node src/nodes/test_state_node.cpp)
ament_target_dependencies(test_state_node PUBLIC
    rclcpp
)
# Link the node against the model library
target_link_libraries(test_state_node PUBLIC
    puzzlebot_planning_model
)

# Specify dependencies for building order
add_dependencies(test_state_node puzzlebot_planning_model)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)

# Install library
install(
    TARGETS puzzlebot_planning_model
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin # Only needed if library has standalone executables
)

# Install the executable
install(
    TARGETS test_state_node
    DESTINATION lib/${PROJECT_NAME}
)

# Export library and includes for downstream packages
ament_export_include_directories(include)
ament_export_libraries(puzzlebot_planning_model)

# Declare the package
ament_package()