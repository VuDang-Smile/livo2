cmake_minimum_required(VERSION 3.10)
project(hba)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17 -O3 -pthread -fexceptions)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find PCL
find_package(PCL REQUIRED COMPONENTS common io filters kdtree segmentation)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find GTSAM (pointing to the local build in dependencies/calibration)
set(GTSAM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../gtsam/build")
find_package(GTSAM REQUIRED QUIET)
include_directories(${GTSAM_INCLUDE_DIR})

# Find Ceres (pointing to the local build in dependencies/calibration)
set(Ceres_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ceres-solver/build")
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

include_directories(include)

# Remove ROS-dependent executables for now, only core hba
add_executable(hba_standalone source/hba.cpp)
target_link_libraries(hba_standalone ${PCL_LIBRARIES} gtsam ceres)

# Define STANDALONE_HBA to wrap out ROS code in hba.cpp
target_compile_definitions(hba_standalone PRIVATE STANDALONE_HBA)

