cmake_minimum_required(VERSION 3.5)
project(livox_msg_converter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  livox_ros_driver2
)

# Add executable
add_executable(livox_msg_converter_node 
  src/livox_msg_converter_node.cpp
)
ament_target_dependencies(livox_msg_converter_node ${dependencies})

# Install executable
install(TARGETS
  livox_msg_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

