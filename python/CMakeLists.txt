find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(
  _kracker
  MODULE
  src/module.cpp
)

target_link_libraries(
  _kracker
  PRIVATE
  kracker::apps
  kracker::tracking
)

target_compile_features(_kracker PRIVATE cxx_std_17)
kracker_apply_project_options(_kracker)

install(
  TARGETS _kracker
  LIBRARY DESTINATION kracker
  RUNTIME DESTINATION kracker
)
