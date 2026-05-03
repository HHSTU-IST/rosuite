include_guard(GLOBAL)

function(kracker_add_test target_name)
  set(options)
  set(one_value_args)
  set(multi_value_args SOURCES LIBRARIES)
  cmake_parse_arguments(
    KRACKER_ADD_TEST
    "${options}"
    "${one_value_args}"
    "${multi_value_args}"
    ${ARGN}
  )

  if(NOT KRACKER_ADD_TEST_SOURCES)
    message(FATAL_ERROR "kracker_add_test(${target_name}) requires SOURCES.")
  endif()

  add_executable("${target_name}" ${KRACKER_ADD_TEST_SOURCES})

  if(KRACKER_ADD_TEST_LIBRARIES)
    target_link_libraries(
      "${target_name}"
      PRIVATE
      ${KRACKER_ADD_TEST_LIBRARIES}
    )
  endif()

  kracker_apply_project_options("${target_name}")
  add_test(NAME "${target_name}" COMMAND "${target_name}")

  set_property(
    GLOBAL
    APPEND
    PROPERTY KRACKER_TEST_TARGETS
    "${target_name}"
  )
endfunction()

function(kracker_init_project_options)
  add_library(kracker_project_options INTERFACE)
  add_library(kracker_project_warnings INTERFACE)

  if(KRACKER_ENABLE_WARNINGS)
    if(MSVC)
      target_compile_options(
        kracker_project_warnings
        INTERFACE
        /W4
        /permissive-
      )
    else()
      target_compile_options(
        kracker_project_warnings
        INTERFACE
        -Wall
        -Wextra
        -Wpedantic
      )
    endif()
  endif()

  if(KRACKER_WARNINGS_AS_ERRORS)
    if(MSVC)
      target_compile_options(kracker_project_warnings INTERFACE /WX)
    else()
      target_compile_options(kracker_project_warnings INTERFACE -Werror)
    endif()
  endif()

  if(KRACKER_ENABLE_SANITIZERS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU")
      set(_kracker_sanitizer_flags "")
      foreach(_sanitizer IN LISTS KRACKER_SANITIZERS)
        if(_sanitizer)
          list(APPEND _kracker_sanitizer_flags "-fsanitize=${_sanitizer}")
        endif()
      endforeach()

      if(_kracker_sanitizer_flags)
        target_compile_options(
          kracker_project_options
          INTERFACE
          ${_kracker_sanitizer_flags}
          -fno-omit-frame-pointer
        )
        target_link_options(
          kracker_project_options
          INTERFACE
          ${_kracker_sanitizer_flags}
          -fno-omit-frame-pointer
        )
      endif()
    else()
      message(WARNING "Sanitizers are only configured for Clang/GNU-family compilers.")
    endif()
  endif()

  if(KRACKER_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU")
      target_compile_options(
        kracker_project_options
        INTERFACE
        --coverage
        -O0
        -g
      )
      target_link_options(kracker_project_options INTERFACE --coverage)
    else()
      message(WARNING "Coverage flags are only configured for Clang/GNU-family compilers.")
    endif()
  endif()

  if(KRACKER_ENABLE_CLANG_TIDY)
    find_program(KRACKER_CLANG_TIDY_EXE NAMES clang-tidy)
    if(KRACKER_CLANG_TIDY_EXE)
      set(CMAKE_CXX_CLANG_TIDY "${KRACKER_CLANG_TIDY_EXE}" PARENT_SCOPE)
    else()
      message(WARNING "KRACKER_ENABLE_CLANG_TIDY is ON, but clang-tidy was not found.")
    endif()
  endif()
endfunction()

function(kracker_apply_project_options target_name)
  get_target_property(_kracker_target_type "${target_name}" TYPE)
  if(_kracker_target_type STREQUAL "INTERFACE_LIBRARY")
    return()
  endif()

  get_target_property(
    _kracker_warning_compile_options
    kracker_project_warnings
    INTERFACE_COMPILE_OPTIONS
  )
  if(_kracker_warning_compile_options)
    target_compile_options(
      "${target_name}"
      PRIVATE
      ${_kracker_warning_compile_options}
    )
  endif()

  get_target_property(
    _kracker_compile_options
    kracker_project_options
    INTERFACE_COMPILE_OPTIONS
  )
  if(_kracker_compile_options)
    target_compile_options(
      "${target_name}"
      PRIVATE
      ${_kracker_compile_options}
    )
  endif()

  get_target_property(
    _kracker_link_options
    kracker_project_options
    INTERFACE_LINK_OPTIONS
  )
  if(_kracker_link_options)
    target_link_options(
      "${target_name}"
      PRIVATE
      ${_kracker_link_options}
    )
  endif()
endfunction()

function(kracker_add_quality_targets)
  file(
    GLOB_RECURSE KRACKER_QUALITY_SOURCES
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

  find_program(KRACKER_CLANG_FORMAT_EXE NAMES clang-format)
  if(KRACKER_CLANG_FORMAT_EXE)
    add_custom_target(
      format
      COMMAND "${KRACKER_CLANG_FORMAT_EXE}" -i ${KRACKER_QUALITY_SOURCES}
      COMMENT "Formatting kracker sources with clang-format"
      VERBATIM
    )
    add_custom_target(
      format-check
      COMMAND "${KRACKER_CLANG_FORMAT_EXE}" --dry-run --Werror ${KRACKER_QUALITY_SOURCES}
      COMMENT "Checking kracker source formatting with clang-format"
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

  if(KRACKER_BUILD_TESTS)
    get_property(
      KRACKER_REGISTERED_TEST_TARGETS
      GLOBAL
      PROPERTY KRACKER_TEST_TARGETS
    )

    add_custom_target(
      check
      COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
      COMMENT "Running kracker test suite"
      VERBATIM
    )

    if(KRACKER_REGISTERED_TEST_TARGETS)
      add_dependencies(check ${KRACKER_REGISTERED_TEST_TARGETS})
    endif()
  else()
    add_custom_target(
      check
      COMMAND "${CMAKE_COMMAND}" -E echo "KRACKER_BUILD_TESTS is OFF; no tests to run."
      VERBATIM
    )
  endif()

  if(KRACKER_ENABLE_COVERAGE)
    find_program(KRACKER_GCOVR_EXE NAMES gcovr)
    if(KRACKER_GCOVR_EXE)
      add_custom_target(
        coverage
        COMMAND
        "${KRACKER_GCOVR_EXE}"
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
      COMMAND "${CMAKE_COMMAND}" -E echo "KRACKER_ENABLE_COVERAGE is OFF; reconfigure with it ON to collect coverage."
      VERBATIM
    )
  endif()
endfunction()
