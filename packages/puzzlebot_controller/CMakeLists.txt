cmake_minimum_required(VERSION 3.8)
project(puzzlebot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(puzzlebot_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

# Add model library
add_library(puzzlebot_controller_include SHARED
    src/include/pure_pursuit.cpp
    src/include/pid_controller.cpp
    src/include/bug0_controller.cpp
)
target_include_directories(puzzlebot_controller_include PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(puzzlebot_controller_include PUBLIC
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    angles
    # Add dependencies like rclcpp if model needs ROS types directly
)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node PUBLIC
    rclcpp
    nav_msgs
    std_msgs
    geometry_msgs
    tf2
    puzzlebot_interfaces
)
# Link the node against the model library
target_link_libraries(controller_node PUBLIC
    puzzlebot_controller_include
)
# Specify dependencies for building order
add_dependencies(controller_node puzzlebot_controller_include)
# Install the main A* node executable
install(
    TARGETS controller_node
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(controller_client src/controller_client.cpp)

ament_target_dependencies(controller_client
  rclcpp
  rclcpp_action
  geometry_msgs
  puzzlebot_interfaces
)

install(TARGETS
  controller_client
  DESTINATION lib/${PROJECT_NAME}
)


# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Install library and executables
install(
    TARGETS puzzlebot_controller_include
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin # Only needed if library has standalone executables
)

# Export library and includes for downstream packages
ament_export_include_directories(include)
ament_export_libraries(puzzlebot_controller_include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
