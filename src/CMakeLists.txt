################################################################
# creation of dynamic graph plugin/entity
################################################################

set(lib_name dg_tools)
# then we build the different entities and install them in devel/lib/plugin folder
# NOTE: in sot-core, these are set as plugins.
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
  data/memory_replay.cpp
  operator.cpp
)

add_library(${lib_name} SHARED ${cpp_source_files})

pkg_config_use_dependency(${lib_name} dynamic-graph)
pkg_config_use_dependency(${lib_name} dynamic-graph-python)
pkg_config_use_dependency(${lib_name} eigen-quadprog)
target_link_libraries(${lib_name} ${catkin_LIBRARIES})
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(${lib_name} rt pthread)
endif()

set(LIBRARIES_TO_BE_EXPORTED ${LIBRARIES_TO_BE_EXPORTED} ${lib_name})

######################################################
# Export the libraries in the devel folder of catkin #
######################################################

set(EXPORTED_LIBRAIRIES ${EXPORTED_LIBRAIRIES} ${LIBRARIES_TO_BE_EXPORTED}
    PARENT_SCOPE
)

####################################
# Export the library as dg plugins #
####################################
set_target_properties(${lib_name} PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${DYNAMIC_GRAPH_PLUGIN_DIR}
)

# dynamic_graph_python_module(
#     from dynamic_graph_manager.dg_tools import *
#     this is the python wrapper dependencies (can be several)
#     This is the actual wrapper target name
# )
dynamic_graph_python_module("${lib_name}"
  ${lib_name}
  ${lib_name}_wrap
)
