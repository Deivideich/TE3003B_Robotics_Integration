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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(puzzlebot_interfaces REQUIRED)

add_library(mcl_utils SHARED
  utils/cpp/mcl_utils.cpp  # <- Replace with your actual .cpp source files
)

# Include path for headers
target_include_directories(mcl_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils/cpp>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Make sure it's position independent (important for Python .so)
set_target_properties(mcl_utils PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# Optional: specify C++ standard
target_compile_features(mcl_utils PUBLIC cxx_std_17)

# Install the compiled .so file into lib/${PROJECT_NAME}
install(TARGETS mcl_utils
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers (optional if you only use .so in Python)
install(DIRECTORY utils/cpp/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h*"  # .h and .hpp
)

# Install directories
install(DIRECTORY scripts rviz maps launch
  DESTINATION share/${PROJECT_NAME})


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