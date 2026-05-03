find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(
  _ros_tracker
  MODULE
    src/module.cpp
)

target_link_libraries(
  _ros_tracker
  PRIVATE
    ros_tracker::apps
    ros_tracker::tracking
)

target_compile_features(_ros_tracker PRIVATE cxx_std_17)
ros_tracker_apply_project_options(_ros_tracker)

install(
  TARGETS _ros_tracker
  LIBRARY DESTINATION ros_tracker
  RUNTIME DESTINATION ros_tracker
)
