add_library(
  ros_tracker_apps
    src/ros.cpp
)
add_library(ros_tracker::apps ALIAS ros_tracker_apps)

set_target_properties(ros_tracker_apps PROPERTIES EXPORT_NAME apps)

ros_tracker_apply_project_options(ros_tracker_apps)

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

install(
  TARGETS ros_tracker_apps
  EXPORT ros_trackerTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

add_executable(
  ros_tracker_offline_example
  example/offline_example.cpp
)
target_link_libraries(ros_tracker_offline_example PRIVATE ros_tracker::apps)
ros_tracker_apply_project_options(ros_tracker_offline_example)

install(
  TARGETS ros_tracker_offline_example
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

if(ROS_TRACKER_BUILD_TESTS)
  add_executable(ros_tracker_apps_tests test/test_apps.cpp)
  target_link_libraries(ros_tracker_apps_tests PRIVATE ros_tracker::apps)
  ros_tracker_apply_project_options(ros_tracker_apps_tests)
  add_test(NAME ros_tracker_apps_tests COMMAND ros_tracker_apps_tests)
endif()
