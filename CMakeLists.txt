cmake_minimum_required(VERSION 3.20)

project(rosuite VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SHARED_LIBS "Build rosuite libraries as shared libraries" OFF)
option(ROSUITE_ENABLE_WARNINGS "Enable project-wide compiler warnings" ON)
option(ROSUITE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(ROSUITE_ENABLE_SANITIZERS "Enable sanitizer instrumentation" OFF)
set(
  ROSUITE_SANITIZERS
  "address;undefined"
  CACHE STRING
  "Semicolon-separated sanitizer list when ROSUITE_ENABLE_SANITIZERS is ON"
)
option(ROSUITE_ENABLE_COVERAGE "Enable coverage instrumentation" OFF)
option(ROSUITE_ENABLE_CLANG_TIDY "Run clang-tidy during compilation when available" OFF)
option(ROSUITE_BUILD_PYTHON "Build rosuite Python bindings" OFF)

option(ROSUITE_BUILD_TESTS "Build rosuite tests" ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(cmake/rosuiteProjectOptions.cmake)

set(ROSUITE_INSTALL_CMAKEDIR
  "${CMAKE_INSTALL_LIBDIR}/cmake/rosuite")

find_package(Eigen3 5.0 REQUIRED NO_MODULE)

if(ROSUITE_BUILD_TESTS)
  enable_testing()
endif()

rosuite_init_project_options()

add_subdirectory(core)
add_subdirectory(models)
add_subdirectory(filters)
add_subdirectory(tracking)
add_subdirectory(apps)

if(ROSUITE_BUILD_PYTHON)
  add_subdirectory(python)
endif()

rosuite_add_quality_targets()

install(
  DIRECTORY
  "${CMAKE_CURRENT_SOURCE_DIR}/core/include/"
  "${CMAKE_CURRENT_SOURCE_DIR}/models/include/"
  "${CMAKE_CURRENT_SOURCE_DIR}/filters/include/"
  "${CMAKE_CURRENT_SOURCE_DIR}/tracking/include/"
  "${CMAKE_CURRENT_SOURCE_DIR}/apps/include/"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/rosuiteConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/rosuiteConfig.cmake"
  INSTALL_DESTINATION "${ROSUITE_INSTALL_CMAKEDIR}"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/rosuiteConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

install(
  EXPORT rosuiteTargets
  FILE rosuiteTargets.cmake
  NAMESPACE rosuite::
  DESTINATION "${ROSUITE_INSTALL_CMAKEDIR}"
)

install(
  FILES
  "${CMAKE_CURRENT_BINARY_DIR}/rosuiteConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/rosuiteConfigVersion.cmake"
  DESTINATION "${ROSUITE_INSTALL_CMAKEDIR}"
)

export(
  EXPORT rosuiteTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/rosuiteTargets.cmake"
  NAMESPACE rosuite::
)
