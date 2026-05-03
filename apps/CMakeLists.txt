add_library(
  kracker_apps
  src/offline_examples.cpp
  src/ros.cpp
)
add_library(kracker::apps ALIAS kracker_apps)

set_target_properties(kracker_apps PROPERTIES EXPORT_NAME apps)

kracker_apply_project_options(kracker_apps)

target_include_directories(
  kracker_apps
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  kracker_apps
  PUBLIC
  kracker::core
  kracker::models
  kracker::filters
  kracker::tracking
)

target_compile_features(kracker_apps PUBLIC cxx_std_17)

install(
  TARGETS kracker_apps
  EXPORT krackerTargets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

add_executable(
  kracker_offline_example
  example/offline_example.cpp
)
target_link_libraries(kracker_offline_example PRIVATE kracker::apps)
kracker_apply_project_options(kracker_offline_example)

install(
  TARGETS kracker_offline_example
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

if(KRACKER_BUILD_TESTS)
  kracker_add_test(
    kracker_apps_tests
    SOURCES test/test_apps.cpp
    LIBRARIES kracker::apps
  )
endif()
