add_library(
  kracker_tracking
  src/association.cpp
  src/base.cpp
  src/management.cpp
  src/model_multi.cpp
  src/tracker.cpp
)
add_library(kracker::tracking ALIAS kracker_tracking)

set_target_properties(kracker_tracking PROPERTIES EXPORT_NAME tracking)

kracker_apply_project_options(kracker_tracking)

target_include_directories(
  kracker_tracking
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  kracker_tracking
  PUBLIC
  kracker::core
  kracker::models
  kracker::filters
)

target_compile_features(kracker_tracking PUBLIC cxx_std_17)

install(
  TARGETS kracker_tracking
  EXPORT krackerTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

if(KRACKER_BUILD_TESTS)
  kracker_add_test(
    kracker_tracking_association_tests
    SOURCES test/test_association.cpp
    LIBRARIES
    kracker::tracking
  )

  kracker_add_test(
    kracker_tracking_tracker_tests
    SOURCES test/test_tracker.cpp
    LIBRARIES
    kracker::tracking
  )

  kracker_add_test(
    kracker_tracking_multi_model_tests
    SOURCES test/test_model_multi.cpp
    LIBRARIES
    kracker::tracking
  )
endif()
