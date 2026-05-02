cmake_minimum_required(VERSION 3.20)

project(ros_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ROS_TRACKER_BUILD_TESTS "Build ros-tracker tests" ON)

find_package(Eigen3 5.0 REQUIRED NO_MODULE)

if(ROS_TRACKER_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(core)
add_subdirectory(models)
add_subdirectory(filters)
