cmake_minimum_required(VERSION 3.20)

project(kracker VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SHARED_LIBS "Build kracker libraries as shared libraries" OFF)
option(KRACKER_ENABLE_WARNINGS "Enable project-wide compiler warnings" ON)
option(KRACKER_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(KRACKER_ENABLE_SANITIZERS "Enable sanitizer instrumentation" OFF)
set(
  KRACKER_SANITIZERS
  "address;undefined"
  CACHE STRING
  "Semicolon-separated sanitizer list when KRACKER_ENABLE_SANITIZERS is ON"
)
option(KRACKER_ENABLE_COVERAGE "Enable coverage instrumentation" OFF)
option(KRACKER_ENABLE_CLANG_TIDY "Run clang-tidy during compilation when available" OFF)
option(KRACKER_BUILD_PYTHON "Build kracker Python bindings" OFF)

option(KRACKER_BUILD_TESTS "Build kracker tests" ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(cmake/krackerProjectOptions.cmake)

set(KRACKER_INSTALL_CMAKEDIR
  "${CMAKE_INSTALL_LIBDIR}/cmake/kracker")

find_package(Eigen3 5.0 REQUIRED NO_MODULE)

if(KRACKER_BUILD_TESTS)
  enable_testing()
endif()

kracker_init_project_options()

add_subdirectory(core)
add_subdirectory(models)
add_subdirectory(filters)
add_subdirectory(tracking)
add_subdirectory(apps)

if(KRACKER_BUILD_PYTHON)
  add_subdirectory(python)
endif()

kracker_add_quality_targets()

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
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/krackerConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/krackerConfig.cmake"
  INSTALL_DESTINATION "${KRACKER_INSTALL_CMAKEDIR}"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/krackerConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

install(
  EXPORT krackerTargets
  FILE krackerTargets.cmake
  NAMESPACE kracker::
  DESTINATION "${KRACKER_INSTALL_CMAKEDIR}"
)

install(
  FILES
  "${CMAKE_CURRENT_BINARY_DIR}/krackerConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/krackerConfigVersion.cmake"
  DESTINATION "${KRACKER_INSTALL_CMAKEDIR}"
)

export(
  EXPORT krackerTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/krackerTargets.cmake"
  NAMESPACE kracker::
)
