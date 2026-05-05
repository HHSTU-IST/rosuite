add_library(
  rosuite_models
  src/motion_models.cpp
)
add_library(rosuite::models ALIAS rosuite_models)

set_target_properties(rosuite_models PROPERTIES EXPORT_NAME models)

rosuite_apply_project_options(rosuite_models)

target_include_directories(
  rosuite_models
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  rosuite_models
  PUBLIC
  rosuite::core
)

target_compile_features(rosuite_models PUBLIC cxx_std_17)

install(
  TARGETS rosuite_models
  EXPORT rosuiteTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

if(ROSUITE_BUILD_TESTS)
  rosuite_add_test(
    rosuite_models_tests
    SOURCES test/test_models.cpp
    LIBRARIES rosuite::models
  )
endif()
