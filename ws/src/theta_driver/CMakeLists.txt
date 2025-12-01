cmake_minimum_required(VERSION 3.5)
project(theta_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost 1.53 REQUIRED system thread timer)
find_package(libuvc REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

find_package(PkgConfig)
pkg_check_modules(GSTREAMER_1_0 gstreamer-1.0)
pkg_check_modules(GSTREAMER_1_0_APP gstreamer-app-1.0)
pkg_check_modules(LibUVC libuvc)

include_directories(
  include
  3rd/libuvc-theta-sample/gst
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_1_0_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  rclcpp_components
)

set(node_plugins "")

# Add library and dependencies
add_library(theta_driver_lib SHARED src/theta_driver_lib.cpp)
rclcpp_components_register_nodes(theta_driver_lib "theta_driver::ThetaDriver")
ament_target_dependencies(theta_driver_lib
  ${dependencies}
  OpenCV
  GSTREAMER_1_0
  GSTREAMER_1_0_APP
  LibUVC
  Boost
)

# Add QR detector library and dependencies
add_library(qr_detector_lib SHARED src/qr_detector_lib.cpp)
rclcpp_components_register_nodes(qr_detector_lib "theta_driver::QRDetector")
ament_target_dependencies(qr_detector_lib
  ${dependencies}
  OpenCV
)
target_link_libraries(qr_detector_lib ${ZBAR_LIBRARIES})

# Add optimized QR detector library and dependencies
add_library(qr_detector_optimized_lib SHARED src/qr_detector_optimized.cpp)
rclcpp_components_register_nodes(qr_detector_optimized_lib "theta_driver::QRDetectorOptimized")
ament_target_dependencies(qr_detector_optimized_lib
  ${dependencies}
  OpenCV
)

# Add perspective converter library and dependencies
add_library(perspective_converter_lib SHARED src/perspective_converter_lib.cpp)
rclcpp_components_register_nodes(perspective_converter_lib "theta_driver::PerspectiveConverter")
ament_target_dependencies(perspective_converter_lib
  ${dependencies}
  OpenCV
)

# Add equidistant converter library and dependencies
add_library(equidistant_converter_lib SHARED src/equidistant_converter_lib.cpp)
rclcpp_components_register_nodes(equidistant_converter_lib "theta_driver::EquidistantConverter")
ament_target_dependencies(equidistant_converter_lib
  ${dependencies}
  OpenCV
)

# Add polynomial converter library and dependencies
add_library(polynomial_converter_lib SHARED src/polynomial_converter_lib.cpp)
rclcpp_components_register_nodes(polynomial_converter_lib "theta_driver::PolynomialConverter")
ament_target_dependencies(polynomial_converter_lib
  ${dependencies}
  OpenCV
)

# Add ATAN converter library and dependencies
add_library(atan_converter_lib SHARED src/atan_converter_lib.cpp)
rclcpp_components_register_nodes(atan_converter_lib "theta_driver::ATANConverter")
ament_target_dependencies(atan_converter_lib
  ${dependencies}
  OpenCV
)

# Add executable and dependencies
add_executable(theta_driver_node 
  src/theta_driver_node.cpp
  3rd/libuvc-theta-sample/gst/thetauvc.c)
target_link_libraries(theta_driver_node theta_driver_lib)
ament_target_dependencies(theta_driver_node ${dependencies})

# Add QR detector executable
add_executable(qr_detector_node src/qr_detector_node.cpp)
target_link_libraries(qr_detector_node qr_detector_lib)
ament_target_dependencies(qr_detector_node ${dependencies})

# Add optimized QR detector executable
add_executable(qr_detector_optimized_node src/qr_detector_optimized_node.cpp)
target_link_libraries(qr_detector_optimized_node qr_detector_optimized_lib)
ament_target_dependencies(qr_detector_optimized_node ${dependencies})

# Add perspective converter executable
add_executable(perspective_converter_node src/perspective_converter_node.cpp)
target_link_libraries(perspective_converter_node perspective_converter_lib)
ament_target_dependencies(perspective_converter_node ${dependencies})

# Add equidistant converter executable
add_executable(equidistant_converter_node src/equidistant_converter_node.cpp)
target_link_libraries(equidistant_converter_node equidistant_converter_lib)
ament_target_dependencies(equidistant_converter_node ${dependencies})

# Add polynomial converter executable
add_executable(polynomial_converter_node src/polynomial_converter_node.cpp)
target_link_libraries(polynomial_converter_node polynomial_converter_lib)
ament_target_dependencies(polynomial_converter_node ${dependencies})

# Add ATAN converter executable
add_executable(atan_converter_node src/atan_converter_node.cpp)
target_link_libraries(atan_converter_node atan_converter_lib)
ament_target_dependencies(atan_converter_node ${dependencies})

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

# Install lib
install(TARGETS
  theta_driver_lib
  qr_detector_lib
  qr_detector_optimized_lib
  perspective_converter_lib
  equidistant_converter_lib
  polynomial_converter_lib
  atan_converter_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
  
# Install exec
install(TARGETS
  theta_driver_node
  qr_detector_node
  qr_detector_optimized_node
  perspective_converter_node
  equidistant_converter_node
  polynomial_converter_node
  atan_converter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ 
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_export_include_directories(
  include
)

    
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
