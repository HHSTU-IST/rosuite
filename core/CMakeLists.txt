add_library(kracker_core INTERFACE)
add_library(kracker::core ALIAS kracker_core)

set_target_properties(kracker_core PROPERTIES EXPORT_NAME core)

target_include_directories(
  kracker_core
  INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(kracker_core INTERFACE Eigen3::Eigen)
target_compile_features(kracker_core INTERFACE cxx_std_17)

install(TARGETS kracker_core EXPORT krackerTargets)

if(KRACKER_BUILD_TESTS)
  kracker_add_test(
    kracker_core_tests
    SOURCES test/test_core.cpp
    LIBRARIES kracker::core
  )
endif()
