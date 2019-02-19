################################################################
# creation of dynamic graph plugin/entity
################################################################

# then we build the different entities and install them in devel/lib/plugin folder
# NOTE: in sot-core, these are set as plugins.
set(LIST_SOURCE_CODE
  control/control_pd.cpp
)

add_library(dg_tools SHARED
  ${LIST_SOURCE_CODE}
)
# TODO change the dependencies to dg and dynamic_graph_python
pkg_config_use_dependency(dg_tools dynamic-graph)
pkg_config_use_dependency(dg_tools dynamic-graph-python)
pkg_config_use_dependency(dg_tools sot-core)
target_link_libraries(dg_tools
  ${catkin_LIBRARIES}
)
# if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
# target_link_libraries(dg_tools
#   rt
#   pthread
# )
# endif(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
set_target_properties(dg_tools PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${DYNAMIC_GRAPH_PLUGIN_DIR}
)

dynamic_graph_python_module("dg_tools"
  dg_tools
  dg_tools_wrap
)
