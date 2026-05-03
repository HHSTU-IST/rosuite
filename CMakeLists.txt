cmake_minimum_required(VERSION 3.20)

project(ros_tracker VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROS_TRACKER_BUILD_TESTS "Build ros-tracker tests" ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(ROS_TRACKER_INSTALL_CMAKEDIR
    "${CMAKE_INSTALL_LIBDIR}/cmake/ros_tracker")

find_package(Eigen3 5.0 REQUIRED NO_MODULE)

if(ROS_TRACKER_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(core)
add_subdirectory(models)
add_subdirectory(filters)
add_subdirectory(tracking)
add_subdirectory(apps)

install(
  DIRECTORY
    "${CMAKE_CURRENT_SOURCE_DIR}/core/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/models/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/filters/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/tracking/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/apps/include/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ros_trackerConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/ros_trackerConfig.cmake"
  INSTALL_DESTINATION "${ROS_TRACKER_INSTALL_CMAKEDIR}"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/ros_trackerConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

install(
  EXPORT ros_trackerTargets
  FILE ros_trackerTargets.cmake
  NAMESPACE ros_tracker::
  DESTINATION "${ROS_TRACKER_INSTALL_CMAKEDIR}"
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ros_trackerConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/ros_trackerConfigVersion.cmake"
  DESTINATION "${ROS_TRACKER_INSTALL_CMAKEDIR}"
)

export(
  EXPORT ros_trackerTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/ros_trackerTargets.cmake"
  NAMESPACE ros_tracker::
)
