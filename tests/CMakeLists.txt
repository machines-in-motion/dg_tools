#
# add unit tests #
#

# create_dg_tools_unittest creates a C++ unit-test over a dynamic-graph plugin.
macro(create_dg_tools_unittest test_name)
  # Test name
  set(cpp_test_name test_${test_name})
  # Create the executable.
  ament_add_gtest(${cpp_test_name} main.cpp ${cpp_test_name}.cpp)
  # Add the include dependencies.
  target_include_directories(
    ${cpp_test_name}
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include>)
  # Link the dependecies to it.
  target_link_libraries(${cpp_test_name} ${test_name})
endmacro(create_dg_tools_unittest test_name)

# C++ unit-tests
find_package(ament_cmake_gtest REQUIRED)
# Create the unit-test.
create_dg_tools_unittest(${PROJECT_NAME}_entities)

# Python unit-tests.
find_package(ament_cmake_nose REQUIRED)
# find all python tests.
file(
  GLOB py_tests
  RELATIVE ${PROJECT_SOURCE_DIR}/tests
  tests/test_*.py)
# Create a test for each one of them.
foreach(py_test ${py_tests})
    get_filename_component(test_name ${py_test} NAME_WE)
    ament_add_nose_test(${PROJECT_NAME}_${py_test} tests/${py_test})
endforeach()
