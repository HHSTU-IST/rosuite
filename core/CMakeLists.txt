add_library(rosuite_core INTERFACE)
add_library(rosuite::core ALIAS rosuite_core)

set_target_properties(rosuite_core PROPERTIES EXPORT_NAME core)

target_include_directories(
  rosuite_core
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rosuite_core INTERFACE Eigen3::Eigen)
target_compile_features(rosuite_core INTERFACE cxx_std_17)

install(TARGETS rosuite_core EXPORT rosuiteTargets)

if(ROSUITE_BUILD_TESTS)
  rosuite_add_test(
    rosuite_core_tests
    SOURCES test/test_core.cpp
    LIBRARIES rosuite::core
  )
endif()
