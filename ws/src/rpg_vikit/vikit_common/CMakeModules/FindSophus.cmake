# FindSophus.cmake
# Tìm Sophus library đã được cài đặt system-wide

# Tìm include directory
# Phiên bản cũ (a621ff) dùng .h, phiên bản mới dùng .hpp
find_path(Sophus_INCLUDE_DIR
  NAMES sophus/so3.hpp sophus/se3.hpp sophus/so3.h sophus/se3.h
  PATHS
    /usr/local/include
    /usr/include
    ${CMAKE_INSTALL_PREFIX}/include
)

# Tìm library
find_library(Sophus_LIBRARY
  NAMES Sophus libSophus
  PATHS
    /usr/local/lib
    /usr/lib
    /usr/local/lib64
    /usr/lib64
    ${CMAKE_INSTALL_PREFIX}/lib
    ${CMAKE_INSTALL_PREFIX}/lib64
)

# Kiểm tra xem đã tìm thấy chưa
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Sophus
  FOUND_VAR Sophus_FOUND
  REQUIRED_VARS Sophus_INCLUDE_DIR Sophus_LIBRARY
)

if(Sophus_FOUND)
  # Tìm Eigen3 (Sophus phụ thuộc vào Eigen)
  find_package(Eigen3 REQUIRED)
  
  # Tạo imported target Sophus::Sophus
  if(NOT TARGET Sophus::Sophus)
    add_library(Sophus::Sophus INTERFACE IMPORTED)
    set_target_properties(Sophus::Sophus PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${Sophus_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES "${Sophus_LIBRARY}"
    )
    
    # Link với Eigen3 nếu có target
    if(TARGET Eigen3::Eigen)
      set_target_properties(Sophus::Sophus PROPERTIES
        INTERFACE_LINK_LIBRARIES "${Sophus_LIBRARY};Eigen3::Eigen"
      )
    else()
      # Nếu không có target, thêm include directory
      get_target_property(EIGEN3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
      if(NOT EIGEN3_INCLUDE_DIRS)
        find_path(EIGEN3_INCLUDE_DIR
          NAMES Eigen/Core
          PATHS /usr/include/eigen3 /usr/local/include/eigen3
        )
        if(EIGEN3_INCLUDE_DIR)
          set_target_properties(Sophus::Sophus PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${Sophus_INCLUDE_DIR};${EIGEN3_INCLUDE_DIR}"
          )
        endif()
      endif()
    endif()
  endif()
  
  # Set variables để tương thích với code cũ
  set(Sophus_INCLUDE_DIRS ${Sophus_INCLUDE_DIR})
  set(Sophus_LIBRARIES ${Sophus_LIBRARY})
  
  message(STATUS "Found Sophus: ${Sophus_INCLUDE_DIR}")
  message(STATUS "Sophus library: ${Sophus_LIBRARY}")
endif()

mark_as_advanced(Sophus_INCLUDE_DIR Sophus_LIBRARY)

