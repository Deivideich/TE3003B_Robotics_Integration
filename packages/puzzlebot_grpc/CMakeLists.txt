cmake_minimum_required(VERSION 3.8)
project(puzzlebot_grpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(puzzlebot_interfaces REQUIRED)


# Generate gRPC Python files into a proper module directory
set(PROTO_DST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/puzzlebot_grpc)

add_custom_command(
  OUTPUT ${PROTO_DST_DIR}/puzzlebot_grpc_pb2.py ${PROTO_DST_DIR}/puzzlebot_grpc_pb2_grpc.py
  COMMAND python3 -m grpc_tools.protoc
          -I${CMAKE_CURRENT_SOURCE_DIR}/protos
          --python_out=${PROTO_DST_DIR}
          --grpc_python_out=${PROTO_DST_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}/protos/puzzlebot_grpc.proto
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/protos/puzzlebot_grpc.proto
)

add_custom_target(proto_py ALL
  DEPENDS
    ${PROTO_DST_DIR}/puzzlebot_grpc_pb2.py
    ${PROTO_DST_DIR}/puzzlebot_grpc_pb2_grpc.py
)

# Install directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

  # Install the generated Python gRPC module
install(DIRECTORY puzzlebot_grpc
  DESTINATION lib/${PROJECT_NAME}
)

# Include every Python script within the src and scripts directory
# # file(GLOB PYTHON_CLASSES src/*.py)
# file(GLOB_RECURSE PYTHON_SCRIPTS scripts/*.py)
# install(PROGRAMS
#   ${PYTHON_SCRIPTS}
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is addOUTPUT ${PROTO_PY}ed to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})
ament_package()

