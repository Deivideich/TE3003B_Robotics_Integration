cmake_minimum_required(VERSION 3.8)
project(puzzlebot_go_grpc)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required ROS build system
find_package(ament_cmake REQUIRED)

# === gRPC Go Code Generation Configuration ===

# Paths
set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/protos)
set(GOOGLEAPIS_DIR /home/robotics/tools/googleapis)
set(GRPC_GATEWAY_PLUGIN /home/robotics/go/bin/protoc-gen-grpc-gateway)

# Proto file(s)
set(PROTO_FILES ${PROTO_DIR}/rpc-puzzlebot-gw.proto)

# Output paths (optional: you can customize to e.g. ${CMAKE_CURRENT_BINARY_DIR}/gen)
set(GO_OUT ${PROTO_DIR})
set(GO_GRPC_OUT ${PROTO_DIR})
set(GRPC_GATEWAY_OUT ${PROTO_DIR})

# Generate Go protobuf and gRPC files
add_custom_command(
  OUTPUT
    ${PROTO_DIR}/rpc-puzzlebot-gw.pb.go
    ${PROTO_DIR}/rpc-puzzlebot-gw_grpc.pb.go
  COMMAND protoc
    -I ${PROTO_DIR}
    -I ${GOOGLEAPIS_DIR}
    --go_out ${GO_OUT} --go_opt paths=source_relative
    --go-grpc_out ${GO_GRPC_OUT} --go-grpc_opt paths=source_relative
    ${PROTO_FILES}
  DEPENDS ${PROTO_FILES}
  COMMENT "Generating Go protobuf and gRPC code"
)

# Generate grpc-gateway reverse proxy
add_custom_command(
  OUTPUT
    ${PROTO_DIR}/rpc-puzzlebot-gw.pb.gw.go
  COMMAND protoc
    -I ${PROTO_DIR}
    -I ${GOOGLEAPIS_DIR}
    --plugin=protoc-gen-grpc-gateway=${GRPC_GATEWAY_PLUGIN}
    --grpc-gateway_out ${GRPC_GATEWAY_OUT}
    --grpc-gateway_opt logtostderr=true
    --grpc-gateway_opt paths=source_relative
    ${PROTO_FILES}
  DEPENDS ${PROTO_FILES}
  COMMENT "Generating gRPC-Gateway reverse proxy"
)

# Define a target to group generation steps
add_custom_target(generate_go_proto ALL
  DEPENDS
    ${PROTO_DIR}/rpc-puzzlebot-gw.pb.go
    ${PROTO_DIR}/rpc-puzzlebot-gw_grpc.pb.go
    ${PROTO_DIR}/rpc-puzzlebot-gw.pb.gw.go
)

set(GO_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(GO_MOD_FILE ${GO_SRC_DIR}/go.mod)
set(GO_MODULE_NAME example.com/puzzlebot_grpc_service)

# Step 1: go mod init
add_custom_command(
  OUTPUT ${GO_MOD_FILE}
  COMMAND ${CMAKE_COMMAND} -E echo "Initializing Go module..."
  COMMAND go mod init ${GO_MODULE_NAME}
  WORKING_DIRECTORY ${GO_SRC_DIR}
  COMMENT "Running 'go mod init ${GO_MODULE_NAME}'"
  VERBATIM
)

add_custom_target(go_mod_init DEPENDS ${GO_MOD_FILE})

# Step 2: go get dependencies
add_custom_command(
  OUTPUT ${GO_SRC_DIR}/.deps_downloaded.stamp
  COMMAND ${CMAKE_COMMAND} -E echo "Fetching Go dependencies..."
  COMMAND go get github.com/golang/glog
  COMMAND go get github.com/grpc-ecosystem/grpc-gateway/v2/runtime
  COMMAND go get google.golang.org/grpc
  COMMAND ${CMAKE_COMMAND} -E touch ${GO_SRC_DIR}/.deps_downloaded.stamp
  WORKING_DIRECTORY ${GO_SRC_DIR}
  DEPENDS go_mod_init
  COMMENT "Downloading Go dependencies"
  VERBATIM
)

add_custom_target(go_get_deps ALL
  DEPENDS ${GO_SRC_DIR}/.deps_downloaded.stamp
)

# Step 3: Compile Go
set(GO_SOURCE ${GO_SRC_DIR}/puzzlebot_grpc_gateway.go)
set(GO_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/puzzlebot_grpc_gateway)

add_custom_command(
  OUTPUT ${GO_OUTPUT}
  COMMAND go build -o ${GO_OUTPUT} ${GO_SOURCE}
  WORKING_DIRECTORY ${GO_SRC_DIR}
  DEPENDS ${GO_SOURCE} go_get_deps
  COMMENT "Building Go gRPC Gateway"
  VERBATIM
)

add_custom_target(build_go_gateway ALL
  DEPENDS ${GO_OUTPUT}
)

# Install the Go binary
install(
  PROGRAMS ${GO_OUTPUT}
  DESTINATION lib/${PROJECT_NAME}
)



# ==============================
# Optionally: Linting for ROS
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Final ROS2 macro
ament_package()
