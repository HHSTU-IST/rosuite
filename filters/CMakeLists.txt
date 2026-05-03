add_library(
  kracker_filters
  src/kalman_support.cpp
  src/kalman_gaussian_transform.cpp
  src/kalman_ensemble.cpp
  src/kalman_h_infinity.cpp
  src/particle_filter.cpp
)
add_library(kracker::filters ALIAS kracker_filters)

set_target_properties(kracker_filters PROPERTIES EXPORT_NAME filters)

kracker_apply_project_options(kracker_filters)

target_include_directories(
  kracker_filters
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  kracker_filters
  PUBLIC
  kracker::core
  kracker::models
)

target_compile_features(kracker_filters PUBLIC cxx_std_17)

install(
  TARGETS kracker_filters
  EXPORT krackerTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

if(KRACKER_BUILD_TESTS)
  kracker_add_test(
    kracker_filters_kalman_tests
    SOURCES test/test_kalman_family.cpp
    LIBRARIES kracker::filters
  )

  kracker_add_test(
    kracker_filters_estimation_tests
    SOURCES test/test_estimation_utils.cpp
    LIBRARIES kracker::filters
  )

  kracker_add_test(
    kracker_filters_particle_tests
    SOURCES test/test_particle.cpp
    LIBRARIES kracker::filters
  )
endif()
