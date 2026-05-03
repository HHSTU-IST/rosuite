add_library(
  ros_tracker_filters
    src/kalman_support.cpp
    src/kalman_gaussian_transform.cpp
    src/kalman_ensemble.cpp
    src/kalman_h_infinity.cpp
    src/particle_filter.cpp
)
add_library(ros_tracker::filters ALIAS ros_tracker_filters)

set_target_properties(ros_tracker_filters PROPERTIES EXPORT_NAME filters)

ros_tracker_apply_project_options(ros_tracker_filters)

target_include_directories(
  ros_tracker_filters
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ros_tracker_filters
  PUBLIC
    ros_tracker::core
    ros_tracker::models
)

target_compile_features(ros_tracker_filters PUBLIC cxx_std_17)

install(
  TARGETS ros_tracker_filters
  EXPORT ros_trackerTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

if(ROS_TRACKER_BUILD_TESTS)
  add_executable(ros_tracker_filters_kalman_tests test/test_kalman_family.cpp)
  target_link_libraries(ros_tracker_filters_kalman_tests PRIVATE ros_tracker::filters)
  ros_tracker_apply_project_options(ros_tracker_filters_kalman_tests)
  add_test(
    NAME ros_tracker_filters_kalman_tests
    COMMAND ros_tracker_filters_kalman_tests
  )

  add_executable(ros_tracker_filters_estimation_tests test/test_estimation_utils.cpp)
  target_link_libraries(ros_tracker_filters_estimation_tests PRIVATE ros_tracker::filters)
  ros_tracker_apply_project_options(ros_tracker_filters_estimation_tests)
  add_test(
    NAME ros_tracker_filters_estimation_tests
    COMMAND ros_tracker_filters_estimation_tests
  )

  add_executable(ros_tracker_filters_particle_tests test/test_particle.cpp)
  target_link_libraries(ros_tracker_filters_particle_tests PRIVATE ros_tracker::filters)
  ros_tracker_apply_project_options(ros_tracker_filters_particle_tests)
  add_test(
    NAME ros_tracker_filters_particle_tests
    COMMAND ros_tracker_filters_particle_tests
  )
endif()
