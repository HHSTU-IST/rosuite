add_library(
  ros_tracker_tracking
  STATIC
    src/association.cpp
    src/base.cpp
    src/management.cpp
    src/tracker.cpp
)
add_library(ros_tracker::tracking ALIAS ros_tracker_tracking)

target_include_directories(
  ros_tracker_tracking
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ros_tracker_tracking
  PUBLIC
    ros_tracker::core
    ros_tracker::models
    ros_tracker::filters
)

target_compile_features(ros_tracker_tracking PUBLIC cxx_std_17)

if(ROS_TRACKER_BUILD_TESTS)
  add_executable(ros_tracker_tracking_tests test/test_tracking.cpp)
  target_link_libraries(ros_tracker_tracking_tests PRIVATE ros_tracker::tracking)
  add_test(NAME ros_tracker_tracking_tests COMMAND ros_tracker_tracking_tests)
endif()
