/**
 * @file test_dg_tools.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-02-19
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <gtest/gtest.h>
#include "dg_tools/control/control_pd.hpp"

/**
 * @brief The DISABLED_TestDGTools class is used to disable test.
 */
class DISABLED_TestDGTools : public ::testing::Test {};

/**
 * @brief The TestDGTools class: test suit template for setting up
 * the unit tests for the Device.
 */
class TestDGTools : public ::testing::Test {
protected:
  /**
   * @brief SetUp, is executed before the unit tests
   */
  void SetUp(){
  }

  /**
   * @brief TearDown, is executed after teh unit tests
   */
  void TearDown() {
  }
};


/**
 * @brief test_start_stop_ros_services, test the start/stop dynamic graph ROS
 * services
 */
TEST_F(TestDGTools, test_control_pd_constructor)
{
  dynamicgraph::sot::ControlPD pd("a_pd_controller");
  ASSERT_EQ("PDController", pd.CLASS_NAME);
  ASSERT_EQ("PDController", pd.getClassName());
  ASSERT_EQ("a_pd_controller", pd.getName());
}