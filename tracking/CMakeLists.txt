add_library(
  ros_tracker_tracking
    src/association.cpp
    src/base.cpp
    src/management.cpp
    src/model_multi.cpp
    src/tracker.cpp
)
add_library(ros_tracker::tracking ALIAS ros_tracker_tracking)

set_target_properties(ros_tracker_tracking PROPERTIES EXPORT_NAME tracking)

ros_tracker_apply_project_options(ros_tracker_tracking)

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

install(
  TARGETS ros_tracker_tracking
  EXPORT ros_trackerTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

if(ROS_TRACKER_BUILD_TESTS)
  ros_tracker_add_test(
    ros_tracker_tracking_association_tests
    SOURCES test/test_association.cpp
    LIBRARIES
      ros_tracker::tracking
  )

  ros_tracker_add_test(
    ros_tracker_tracking_tracker_tests
    SOURCES test/test_tracker.cpp
    LIBRARIES
      ros_tracker::tracking
  )

  ros_tracker_add_test(
    ros_tracker_tracking_multi_model_tests
    SOURCES test/test_model_multi.cpp
    LIBRARIES
      ros_tracker::tracking
  )
endif()
