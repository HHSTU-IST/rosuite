add_library(
  rosuite_apps
  src/offline_examples.cpp
  src/ros.cpp
)
add_library(rosuite::apps ALIAS rosuite_apps)

set_target_properties(rosuite_apps PROPERTIES EXPORT_NAME apps)

rosuite_apply_project_options(rosuite_apps)

target_include_directories(
  rosuite_apps
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  rosuite_apps
  PUBLIC
  rosuite::core
  rosuite::models
  rosuite::filters
  rosuite::tracking
)

target_compile_features(rosuite_apps PUBLIC cxx_std_17)

install(
  TARGETS rosuite_apps
  EXPORT rosuiteTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

add_executable(
  rosuite_offline_example
  example/offline_example.cpp
)
target_link_libraries(rosuite_offline_example PRIVATE rosuite::apps)
rosuite_apply_project_options(rosuite_offline_example)

install(
  TARGETS rosuite_offline_example
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

if(ROSUITE_BUILD_TESTS)
  rosuite_add_test(
    rosuite_apps_tests
    SOURCES test/test_apps.cpp
    LIBRARIES rosuite::apps
  )
endif()
