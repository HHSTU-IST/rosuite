add_library(
  ros_tracker_apps
  STATIC
    src/ros.cpp
)
add_library(ros_tracker::apps ALIAS ros_tracker_apps)

target_include_directories(
  ros_tracker_apps
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ros_tracker_apps
  PUBLIC
    ros_tracker::core
    ros_tracker::models
    ros_tracker::filters
    ros_tracker::tracking
)

target_compile_features(ros_tracker_apps PUBLIC cxx_std_17)

add_executable(
  ros_tracker_offline_example
  example/offline_example.cpp
)
target_link_libraries(ros_tracker_offline_example PRIVATE ros_tracker::apps)

if(ROS_TRACKER_BUILD_TESTS)
  add_executable(ros_tracker_apps_tests test/test_apps.cpp)
  target_link_libraries(ros_tracker_apps_tests PRIVATE ros_tracker::apps)
  add_test(NAME ros_tracker_apps_tests COMMAND ros_tracker_apps_tests)
endif()
