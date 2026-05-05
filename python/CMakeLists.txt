find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(
  _rosuite
  MODULE
  src/module.cpp
)

target_link_libraries(
  _rosuite
  PRIVATE
  rosuite::apps
  rosuite::tracking
)

target_compile_features(_rosuite PRIVATE cxx_std_17)
rosuite_apply_project_options(_rosuite)

install(
  TARGETS _rosuite
  LIBRARY DESTINATION rosuite
  RUNTIME DESTINATION rosuite
)
