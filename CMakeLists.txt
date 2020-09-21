######################
# set up the project #
######################
cmake_minimum_required(VERSION 2.8.3)

project(dg_tools)

# required to use std::shared_ptr in code and to link the python bindings
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed")
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# ensuring path to libraries are set during install
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

############################
# Search for dependencies. #
############################

set(CATKIN_PKGS ${CATKIN_PKGS}
    mpi_cmake_modules
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PKGS})

search_for_eigen()
search_for_python()

# Trigger dependency to dynamic-graph.
#add_required_dependency("eigen-quadprog")
add_required_dependency("dynamic-graph >= 3.0.0")
add_required_dependency("dynamic-graph-python >= 3.0.0")

######################################################
# define the include directory of all ${CATKIN_PKGS} #
######################################################
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()


########################################################
# manage the creation of the libraries and executables #
########################################################
add_subdirectory(src)

##########################################
# export the package as a catkin package #
##########################################
catkin_package(
  LIBRARIES ${EXPORTED_LIBRAIRIES}
  INCLUDE_DIRS include
)

#########################
# manage the unit tests #
#########################
add_subdirectory(tests)

####################
# manage the demos #
####################
add_subdirectory(demos)


##########################
# building documentation #
##########################
build_doxygen_documentation()
