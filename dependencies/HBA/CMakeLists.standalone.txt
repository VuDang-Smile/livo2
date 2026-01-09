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

# Find GTSAM (pointing to the local build in dependencies/calibration or project root)
if(NOT GTSAM_DIR)
    find_path(GTSAM_LOCAL_DIR NAMES gtsam/base/Vector.h
        PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../calibration/gtsam"
              "${CMAKE_CURRENT_SOURCE_DIR}/../gtsam"
              "${CMAKE_CURRENT_SOURCE_DIR}/../../dependencies/calibration/gtsam"
    )
    if(GTSAM_LOCAL_DIR)
        set(GTSAM_DIR "${GTSAM_LOCAL_DIR}/build")
    endif()
endif()
find_package(GTSAM REQUIRED QUIET)
include_directories(${GTSAM_INCLUDE_DIR})

# Find Ceres (pointing to the local build in dependencies/calibration or project root)
if(NOT Ceres_DIR)
    find_path(CERES_LOCAL_DIR NAMES ceres/ceres.h
        PATHS "${CMAKE_CURRENT_SOURCE_DIR}/../calibration/ceres-solver"
              "${CMAKE_CURRENT_SOURCE_DIR}/../ceres-solver"
              "${CMAKE_CURRENT_SOURCE_DIR}/../../dependencies/calibration/ceres-solver"
    )
    if(CERES_LOCAL_DIR)
        set(Ceres_DIR "${CERES_LOCAL_DIR}/build")
    endif()
endif()
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

include_directories(include)

# Remove ROS-dependent executables for now, only core hba
add_executable(hba_standalone source/hba.cpp)
target_link_libraries(hba_standalone ${PCL_LIBRARIES} gtsam ceres)

# Define STANDALONE_HBA to wrap out ROS code in hba.cpp
target_compile_definitions(hba_standalone PRIVATE STANDALONE_HBA)

