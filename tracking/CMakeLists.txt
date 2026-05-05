add_library(
  rosuite_tracking
  src/association.cpp
  src/base.cpp
  src/management.cpp
  src/model_multi.cpp
  src/tracker.cpp
)
add_library(rosuite::tracking ALIAS rosuite_tracking)

set_target_properties(rosuite_tracking PROPERTIES EXPORT_NAME tracking)

rosuite_apply_project_options(rosuite_tracking)

target_include_directories(
  rosuite_tracking
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  rosuite_tracking
  PUBLIC
  rosuite::core
  rosuite::models
  rosuite::filters
)

target_compile_features(rosuite_tracking PUBLIC cxx_std_17)

install(
  TARGETS rosuite_tracking
  EXPORT rosuiteTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

if(ROSUITE_BUILD_TESTS)
  rosuite_add_test(
    rosuite_tracking_association_tests
    SOURCES test/test_association.cpp
    LIBRARIES
    rosuite::tracking
  )

  rosuite_add_test(
    rosuite_tracking_tracker_tests
    SOURCES test/test_tracker.cpp
    LIBRARIES
    rosuite::tracking
  )

  rosuite_add_test(
    rosuite_tracking_multi_model_tests
    SOURCES test/test_model_multi.cpp
    LIBRARIES
    rosuite::tracking
  )
endif()
