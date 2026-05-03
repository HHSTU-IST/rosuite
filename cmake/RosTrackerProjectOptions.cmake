include_guard(GLOBAL)

function(ros_tracker_add_test target_name)
  set(options)
  set(one_value_args)
  set(multi_value_args SOURCES LIBRARIES)
  cmake_parse_arguments(
    ROS_TRACKER_ADD_TEST
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN}
  )

  if(NOT ROS_TRACKER_ADD_TEST_SOURCES)
    message(FATAL_ERROR "ros_tracker_add_test(${target_name}) requires SOURCES.")
  endif()

  add_executable("${target_name}" ${ROS_TRACKER_ADD_TEST_SOURCES})

  if(ROS_TRACKER_ADD_TEST_LIBRARIES)
    target_link_libraries(
      "${target_name}"
      PRIVATE
        ${ROS_TRACKER_ADD_TEST_LIBRARIES}
    )
  endif()

  ros_tracker_apply_project_options("${target_name}")
  add_test(NAME "${target_name}" COMMAND "${target_name}")

  set_property(
    GLOBAL
    APPEND
    PROPERTY ROS_TRACKER_TEST_TARGETS
    "${target_name}"
  )
endfunction()

function(ros_tracker_init_project_options)
  add_library(ros_tracker_project_options INTERFACE)
  add_library(ros_tracker_project_warnings INTERFACE)

  if(ROS_TRACKER_ENABLE_WARNINGS)
    if(MSVC)
      target_compile_options(
        ros_tracker_project_warnings
        INTERFACE
          /W4
          /permissive-
      )
    else()
      target_compile_options(
        ros_tracker_project_warnings
        INTERFACE
          -Wall
          -Wextra
          -Wpedantic
      )
    endif()
  endif()

  if(ROS_TRACKER_WARNINGS_AS_ERRORS)
    if(MSVC)
      target_compile_options(ros_tracker_project_warnings INTERFACE /WX)
    else()
      target_compile_options(ros_tracker_project_warnings INTERFACE -Werror)
    endif()
  endif()

  if(ROS_TRACKER_ENABLE_SANITIZERS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU")
      set(_ros_tracker_sanitizer_flags "")
      foreach(_sanitizer IN LISTS ROS_TRACKER_SANITIZERS)
        if(_sanitizer)
          list(APPEND _ros_tracker_sanitizer_flags "-fsanitize=${_sanitizer}")
        endif()
      endforeach()

      if(_ros_tracker_sanitizer_flags)
        target_compile_options(
          ros_tracker_project_options
          INTERFACE
            ${_ros_tracker_sanitizer_flags}
            -fno-omit-frame-pointer
        )
        target_link_options(
          ros_tracker_project_options
          INTERFACE
            ${_ros_tracker_sanitizer_flags}
            -fno-omit-frame-pointer
        )
      endif()
    else()
      message(WARNING "Sanitizers are only configured for Clang/GNU-family compilers.")
    endif()
  endif()

  if(ROS_TRACKER_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU")
      target_compile_options(
        ros_tracker_project_options
        INTERFACE
          --coverage
          -O0
          -g
      )
      target_link_options(ros_tracker_project_options INTERFACE --coverage)
    else()
      message(WARNING "Coverage flags are only configured for Clang/GNU-family compilers.")
    endif()
  endif()

  if(ROS_TRACKER_ENABLE_CLANG_TIDY)
    find_program(ROS_TRACKER_CLANG_TIDY_EXE NAMES clang-tidy)
    if(ROS_TRACKER_CLANG_TIDY_EXE)
      set(CMAKE_CXX_CLANG_TIDY "${ROS_TRACKER_CLANG_TIDY_EXE}" PARENT_SCOPE)
    else()
      message(WARNING "ROS_TRACKER_ENABLE_CLANG_TIDY is ON, but clang-tidy was not found.")
    endif()
  endif()
endfunction()

function(ros_tracker_apply_project_options target_name)
  get_target_property(_ros_tracker_target_type "${target_name}" TYPE)
  if(_ros_tracker_target_type STREQUAL "INTERFACE_LIBRARY")
    return()
  endif()

  get_target_property(
    _ros_tracker_warning_compile_options
    ros_tracker_project_warnings
    INTERFACE_COMPILE_OPTIONS
  )
  if(_ros_tracker_warning_compile_options)
    target_compile_options(
      "${target_name}"
      PRIVATE
        ${_ros_tracker_warning_compile_options}
    )
  endif()

  get_target_property(
    _ros_tracker_compile_options
    ros_tracker_project_options
    INTERFACE_COMPILE_OPTIONS
  )
  if(_ros_tracker_compile_options)
    target_compile_options(
      "${target_name}"
      PRIVATE
        ${_ros_tracker_compile_options}
    )
  endif()

  get_target_property(
    _ros_tracker_link_options
    ros_tracker_project_options
    INTERFACE_LINK_OPTIONS
  )
  if(_ros_tracker_link_options)
    target_link_options(
      "${target_name}"
      PRIVATE
        ${_ros_tracker_link_options}
    )
  endif()
endfunction()

function(ros_tracker_add_quality_targets)
  file(
    GLOB_RECURSE ROS_TRACKER_QUALITY_SOURCES
    CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/core/include/*.hpp"
    "${CMAKE_SOURCE_DIR}/core/test/*.cpp"
    "${CMAKE_SOURCE_DIR}/models/include/*.hpp"
    "${CMAKE_SOURCE_DIR}/models/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/models/test/*.cpp"
    "${CMAKE_SOURCE_DIR}/filters/include/*.hpp"
    "${CMAKE_SOURCE_DIR}/filters/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/filters/test/*.cpp"
    "${CMAKE_SOURCE_DIR}/tracking/include/*.hpp"
    "${CMAKE_SOURCE_DIR}/tracking/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/tracking/test/*.cpp"
    "${CMAKE_SOURCE_DIR}/apps/include/*.hpp"
    "${CMAKE_SOURCE_DIR}/apps/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/apps/test/*.cpp"
    "${CMAKE_SOURCE_DIR}/apps/example/*.cpp"
  )

  find_program(ROS_TRACKER_CLANG_FORMAT_EXE NAMES clang-format)
  if(ROS_TRACKER_CLANG_FORMAT_EXE)
    add_custom_target(
      format
      COMMAND "${ROS_TRACKER_CLANG_FORMAT_EXE}" -i ${ROS_TRACKER_QUALITY_SOURCES}
      COMMENT "Formatting ros-tracker sources with clang-format"
      VERBATIM
    )
    add_custom_target(
      format-check
      COMMAND "${ROS_TRACKER_CLANG_FORMAT_EXE}" --dry-run --Werror ${ROS_TRACKER_QUALITY_SOURCES}
      COMMENT "Checking ros-tracker source formatting with clang-format"
      VERBATIM
    )
  else()
    add_custom_target(
      format
      COMMAND "${CMAKE_COMMAND}" -E echo "clang-format not found; install it to use the format target."
      VERBATIM
    )
    add_custom_target(
      format-check
      COMMAND "${CMAKE_COMMAND}" -E echo "clang-format not found; install it to use the format-check target."
      VERBATIM
    )
  endif()

  add_custom_target(lint DEPENDS format-check)

  if(ROS_TRACKER_BUILD_TESTS)
    get_property(
      ROS_TRACKER_REGISTERED_TEST_TARGETS
      GLOBAL
      PROPERTY ROS_TRACKER_TEST_TARGETS
    )

    add_custom_target(
      check
      COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
      COMMENT "Running ros-tracker test suite"
      VERBATIM
    )

    if(ROS_TRACKER_REGISTERED_TEST_TARGETS)
      add_dependencies(check ${ROS_TRACKER_REGISTERED_TEST_TARGETS})
    endif()
  else()
    add_custom_target(
      check
      COMMAND "${CMAKE_COMMAND}" -E echo "ROS_TRACKER_BUILD_TESTS is OFF; no tests to run."
      VERBATIM
    )
  endif()

  if(ROS_TRACKER_ENABLE_COVERAGE)
    find_program(ROS_TRACKER_GCOVR_EXE NAMES gcovr)
    if(ROS_TRACKER_GCOVR_EXE)
      add_custom_target(
        coverage
        COMMAND
          "${ROS_TRACKER_GCOVR_EXE}"
          --root "${CMAKE_SOURCE_DIR}"
          --exclude "${CMAKE_BINARY_DIR}"
          --txt
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
        COMMENT "Generating coverage report with gcovr"
        VERBATIM
      )
    else()
      add_custom_target(
        coverage
        COMMAND "${CMAKE_COMMAND}" -E echo "gcovr not found; install it to use the coverage target."
        VERBATIM
      )
    endif()
  else()
    add_custom_target(
      coverage
      COMMAND "${CMAKE_COMMAND}" -E echo "ROS_TRACKER_ENABLE_COVERAGE is OFF; reconfigure with it ON to collect coverage."
      VERBATIM
    )
  endif()
endfunction()
