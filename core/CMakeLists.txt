add_library(ros_tracker_core INTERFACE)
add_library(ros_tracker::core ALIAS ros_tracker_core)

set_target_properties(ros_tracker_core PROPERTIES EXPORT_NAME core)

target_include_directories(
  ros_tracker_core
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(ros_tracker_core INTERFACE Eigen3::Eigen)
target_compile_features(ros_tracker_core INTERFACE cxx_std_17)

install(TARGETS ros_tracker_core EXPORT ros_trackerTargets)

if(ROS_TRACKER_BUILD_TESTS)
  ros_tracker_add_test(
    ros_tracker_core_tests
    SOURCES test/test_core.cpp
    LIBRARIES ros_tracker::core
  )
endif()
