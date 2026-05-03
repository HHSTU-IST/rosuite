find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(
  _Kracker
  MODULE
  src/module.cpp
)

target_link_libraries(
  _Kracker
  PRIVATE
  kracker::apps
  kracker::tracking
)

target_compile_features(_Kracker PRIVATE cxx_std_17)
kracker_apply_project_options(_Kracker)

install(
  TARGETS _Kracker
  LIBRARY DESTINATION Kracker
  RUNTIME DESTINATION Kracker
)
