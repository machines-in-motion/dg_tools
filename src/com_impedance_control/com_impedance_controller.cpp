// impedance controller implementation for COM (used for quadruped)
// Author : Avadesh Meduri
// Date : 25/03/19

#include <iostream>
#include "dg_tools/com_impedance_control/com_impedance_controller.hpp"


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
using namespace std;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(com_impedance_control, "com_impedance_control");


com_impedance_control::com_impedance_control(const std::string & name)
  :Entity(name)
  ,TimeStep(0)
  ,KpSIN(NULL, "com_impedance_control("+name+")::input(vector)::Kp")
  ,KdSIN(NULL, "com_impedance_control("+name+")::input(vector)::Kd")
  ,positionSIN(NULL, "com_impedance_control("+name+")::input(vector)::position")
  ,desiredpositionSIN(NULL, "com_impedance_control("+name+")::input(vector)::desired_position")
  ,velocitySIN(NULL, "com_impedance_control("+name+")::input(vector)::velocity")
  ,desiredvelocitySIN(NULL, "com_impedance_control("+name+")::input(vector)::desired_velocity")
  ,controlSOUT( boost::bind(&com_impedance_control::return_control_torques, this, _1,_2),
                KpSIN << KdSIN << positionSIN << desiredpositionSIN <<
                velocitySIN << desiredvelocitySIN,
                "com_impedance_control("+name+")::output(vector)::control")

{
  init(TimeStep);
  Entity::signalRegistration(
    KpSIN << KdSIN << positionSIN << desiredpositionSIN <<
    velocitySIN << desiredvelocitySIN << controlSOUT
  );
}

dynamicgraph::Vector& com_impedance_control::
  return_control_torques( dynamicgraph::Vector &tau, int t){
    sotDEBUGIN(15);
    const dynamicgraph::Vector& Kp = KpSIN(t);
    // const dynamicgraph::Vector& Kd = KdSIN(t);
    const dynamicgraph::Vector& position = positionSIN(t);
    const dynamicgraph::Vector& des_pos = desiredpositionSIN(t);
    // const dynamicgraph::Vector& velocity = velocitySIN(t);
    // const dynamicgraph::Vector& des_vel = desiredvelocitySIN(t);

    /*----- assertions of sizes -------------*/
    assert(Kp.size() == 7);
    // assert(Kd.size() == 6);
    assert(position.size() == 7);
    assert(des_pos.size() == 7);
    // assert(velocity.size() == 6);
    // assert(des_vel.size() == 6);

    /*---------- computing position error ----*/

    pos_error.array() = des_pos.array() - position.array();
    //vel_error.array() = des_vel.array() - velocity.array();

    cout << "pos_error is" << pos_error[2] << endl;

    tau.array() = pos_error.array()*Kp.array();
                  //+ vel_error.array()*Kd.array();

    sotDEBUGOUT(15);

    return tau;

  }
