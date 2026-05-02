add_library(ros_tracker_filters INTERFACE)
add_library(ros_tracker::filters ALIAS ros_tracker_filters)

target_include_directories(
  ros_tracker_filters
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ros_tracker_filters
  INTERFACE
    ros_tracker::core
    ros_tracker::models
)

target_compile_features(ros_tracker_filters INTERFACE cxx_std_17)

if(ROS_TRACKER_BUILD_TESTS)
  add_executable(ros_tracker_filters_tests test/test_filters.cpp)
  target_link_libraries(ros_tracker_filters_tests PRIVATE ros_tracker::filters)
  add_test(NAME ros_tracker_filters_tests COMMAND ros_tracker_filters_tests)
endif()
