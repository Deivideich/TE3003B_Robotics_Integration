cmake_minimum_required(VERSION 3.8)
project(puzzlebot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf_launch REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(puzzlebot_interfaces REQUIRED)

# Add model library
add_library(puzzlebot_navigation_include SHARED
    src/include/core_slam.cpp
)
target_include_directories(puzzlebot_navigation_include PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Link model library against necessary dependencies (if any, none for now)
ament_target_dependencies(puzzlebot_navigation_include PUBLIC
    # Add dependencies like rclcpp if model needs ROS types directly
)

# MAIN EXECUTABLES
# Add the main  A* service node executable
add_executable(core_slam_node src/core_slam_node.cpp)
ament_target_dependencies(core_slam_node PUBLIC
    rclcpp
    nav_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
    puzzlebot_interfaces
)
# Link the node against the model library
target_link_libraries(core_slam_node PUBLIC
    puzzlebot_navigation_include
)
# Specify dependencies for building order
add_dependencies(core_slam_node puzzlebot_navigation_include)
# Install the main A* node executable
install(
    TARGETS core_slam_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY scripts rviz maps launch
  DESTINATION share/${PROJECT_NAME})

  # Install shared C++ library
install(
  DIRECTORY utils/cpp/
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.so"
)


# Include every Python script within the src and scripts directory
# file(GLOB PYTHON_CLASSES src/*.py)
file(GLOB_RECURSE PYTHON_SCRIPTS scripts/*.py)
install(PROGRAMS
  ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# Python compatability
ament_python_install_package(${PROJECT_NAME})

# #For testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()