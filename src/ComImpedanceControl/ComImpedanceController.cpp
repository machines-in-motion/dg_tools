// impedance controller implementation for COM (used for quadruped)
// Author : Avadesh Meduri
// Date : 25/03/19

#include <iostream>
#include "dg_tools/ComImpedanceControl/ComImpedanceController.hpp"


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
using namespace std;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ComImpedanceControl, "ComImpedanceControl");


ComImpedanceControl::ComImpedanceControl(const std::string & name)
  :Entity(name)
  ,TimeStep(0)
  ,KpSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::Kp")
  ,KdSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::Kd")
  ,positionSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::position")
  ,desiredpositionSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_pos")
  ,velocitySIN(NULL, "ComImpedanceControl("+name+")::input(vector)::velocity")
  ,desiredvelocitySIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_vel")
  ,feedforwardforceSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_fff")
  ,controlSOUT( boost::bind(&ComImpedanceControl::return_control_torques, this, _1,_2),
                KpSIN << KdSIN << positionSIN << desiredpositionSIN <<
                velocitySIN << desiredvelocitySIN,
                "ComImpedanceControl("+name+")::output(vector)::tau")
  ,pos_biasSOUT( boost::bind(&ComImpedanceControl::set_bias, this, _1, _2, _3),
                positionSIN << velocitySIN, "ComImpedanceControl("+name+")::output(vector)::issetbias"

  )

{
  init(TimeStep);
  Entity::signalRegistration(
    KpSIN << KdSIN << positionSIN << desiredpositionSIN <<
    velocitySIN << desiredvelocitySIN << feedforwardforceSIN << controlSOUT
  );
}

dynamicgraph::Vector& ComImpedanceControl::
  return_control_torques( dynamicgraph::Vector &tau, int t){
    sotDEBUGIN(15);
    const dynamicgraph::Vector& Kp = KpSIN(t);
    const dynamicgraph::Vector& Kd = KdSIN(t);
    const dynamicgraph::Vector& position = positionSIN(t);
    const dynamicgraph::Vector& des_pos = desiredpositionSIN(t);
    const dynamicgraph::Vector& velocity = velocitySIN(t);
    const dynamicgraph::Vector& des_vel = desiredvelocitySIN(t);
    const dynamicgraph::Vector& des_fff = feedforwardforceSIN(t);

    /*----- assertions of sizes -------------*/
    assert(Kp.size() == 3);
    assert(Kd.size() == 3);
    assert(position.size() == 3);
    assert(des_pos.size() == 3);
    assert(velocity.size() == 3);
    assert(des_vel.size() == 3);
    assert(des_fff.size() == 3);

    /*---------- checking if position is biased */

    /*---------- computing position error ----*/
    pos_error.array() = des_pos.array() - position.array();
    vel_error.array() = des_vel.array() - velocity.array();
    tau.array() = des_fff.array() + pos_error.array()*Kp.array()
                  + vel_error.array()*Kd.array();

    sotDEBUGOUT(15);

    return tau;

  }

bool ComImpedanceControl::
  set_bias(dynamicgraph::Vector& pos_bias, dynamicgraph::Vector& vel_bias, int t){

    const dynamicgraph::Vector& position = positionSIN(t);
    const dynamicgraph::Vector& velocity = velocitySIN(t);

    if (t == 0){
      vicon_pos_bias.array() = position.array();
      vicon_vel_bias.array() = velocity.array();
    }
    else if(t < 1000){
        vicon_pos_bias.array() += position.array();
        vicon_vel_bias.array() += velocity.array();
      }
    else {
      vicon_pos_bias.array() = 0.001 * vicon_pos_bias.array();
      vicon_vel_bias.array() = 0.001 * vicon_vel_bias.array();

    }
    return true;


}
