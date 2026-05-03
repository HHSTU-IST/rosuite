add_library(
  ros_tracker_models
    src/motion_models.cpp
)
add_library(ros_tracker::models ALIAS ros_tracker_models)

set_target_properties(ros_tracker_models PROPERTIES EXPORT_NAME models)

ros_tracker_apply_project_options(ros_tracker_models)

target_include_directories(
  ros_tracker_models
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ros_tracker_models
  PUBLIC
    ros_tracker::core
)

target_compile_features(ros_tracker_models PUBLIC cxx_std_17)

install(
  TARGETS ros_tracker_models
  EXPORT ros_trackerTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

if(ROS_TRACKER_BUILD_TESTS)
  ros_tracker_add_test(
    ros_tracker_models_tests
    SOURCES test/test_models.cpp
    LIBRARIES ros_tracker::models
  )
endif()
