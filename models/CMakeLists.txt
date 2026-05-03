add_library(
  kracker_models
  src/motion_models.cpp
)
add_library(kracker::models ALIAS kracker_models)

set_target_properties(kracker_models PROPERTIES EXPORT_NAME models)

kracker_apply_project_options(kracker_models)

target_include_directories(
  kracker_models
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  kracker_models
  PUBLIC
  kracker::core
)

target_compile_features(kracker_models PUBLIC cxx_std_17)

install(
  TARGETS kracker_models
  EXPORT krackerTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

if(ROS_TRACKER_BUILD_TESTS)
  kracker_add_test(
    kracker_models_tests
    SOURCES test/test_models.cpp
    LIBRARIES kracker::models
  )
endif()
