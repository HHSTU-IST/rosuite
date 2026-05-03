add_library(ros_tracker_models INTERFACE)
add_library(ros_tracker::models ALIAS ros_tracker_models)

set_target_properties(ros_tracker_models PROPERTIES EXPORT_NAME models)

target_include_directories(
  ros_tracker_models
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ros_tracker_models
  INTERFACE
    ros_tracker::core
)

target_compile_features(ros_tracker_models INTERFACE cxx_std_17)

install(TARGETS ros_tracker_models EXPORT ros_trackerTargets)

if(ROS_TRACKER_BUILD_TESTS)
  add_executable(ros_tracker_models_tests test/test_models.cpp)
  target_link_libraries(ros_tracker_models_tests PRIVATE ros_tracker::models)
  add_test(NAME ros_tracker_models_tests COMMAND ros_tracker_models_tests)
endif()
