#
# Creation of dynamic graph plugin/entity.
#

set(lib_name ${PROJECT_NAME}_entities)
# then we build the different entities in a dynamic-graph plugin.
set(cpp_source_files
  com_impedance_control/com_impedance_controller.cpp
  com_impedance_control/reactive_lqr_controller.cpp
  test_stand_control/power_jump.cpp
  control/control_pd.cpp
  control/calibrator.cpp
  data/history_recorder.cpp
  data/upsampler.cpp
  data/previous_value.cpp
  data/memory_replay.cpp
  operator.cpp
  smooth_reach.cpp
  se3_offset.cpp
)

add_library(${lib_name} SHARED ${cpp_source_files})
# Add the include dependencies.
target_include_directories(
  ${lib_name} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)
# Dependencies.
target_link_libraries(${lib_name} dynamic-graph::dynamic-graph)
target_link_libraries(${lib_name} dynamic-graph-python::dynamic-graph-python)
target_link_libraries(${lib_name} sot-core::sot-core)
target_link_libraries(${lib_name} pinocchio::pinocchio)
target_link_libraries(${lib_name} Eigen3::Eigen)
target_link_libraries(${lib_name} eigen-quadprog::eigen-quadprog)
# Install the plugin python bindings.
install_dynamic_graph_plugin_python_bindings(${lib_name})
# Install the plugin.
get_dynamic_graph_plugin_install_path(plugin_install_path)
install(
  TARGETS ${lib_name}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION ${plugin_install_path}
  ARCHIVE DESTINATION ${plugin_install_path}
  RUNTIME DESTINATION ${plugin_install_path}
  INCLUDES
  DESTINATION include)
