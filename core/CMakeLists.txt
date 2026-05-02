add_library(ros_tracker_core INTERFACE)
add_library(ros_tracker::core ALIAS ros_tracker_core)

target_include_directories(
  ros_tracker_core
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(ros_tracker_core INTERFACE Eigen3::Eigen)
target_compile_features(ros_tracker_core INTERFACE cxx_std_20)

if(ROS_TRACKER_BUILD_TESTS)
  add_executable(ros_tracker_core_tests test/test_core.cpp)
  target_link_libraries(ros_tracker_core_tests PRIVATE ros_tracker::core)
  add_test(NAME ros_tracker_core_tests COMMAND ros_tracker_core_tests)
endif()
