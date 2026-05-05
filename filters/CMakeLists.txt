add_library(
  rosuite_filters
  src/kalman_support.cpp
  src/kalman_gaussian_transform.cpp
  src/kalman_ensemble.cpp
  src/kalman_h_infinity.cpp
  src/particle_filter.cpp
)
add_library(rosuite::filters ALIAS rosuite_filters)

set_target_properties(rosuite_filters PROPERTIES EXPORT_NAME filters)

rosuite_apply_project_options(rosuite_filters)

target_include_directories(
  rosuite_filters
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  rosuite_filters
  PUBLIC
  rosuite::core
  rosuite::models
)

target_compile_features(rosuite_filters PUBLIC cxx_std_17)

install(
  TARGETS rosuite_filters
  EXPORT rosuiteTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

if(ROSUITE_BUILD_TESTS)
  rosuite_add_test(
    rosuite_filters_kalman_tests
    SOURCES test/test_kalman_family.cpp
    LIBRARIES rosuite::filters
  )

  rosuite_add_test(
    rosuite_filters_estimation_tests
    SOURCES test/test_estimation_utils.cpp
    LIBRARIES rosuite::filters
  )

  rosuite_add_test(
    rosuite_filters_particle_tests
    SOURCES test/test_particle.cpp
    LIBRARIES rosuite::filters
  )
endif()
