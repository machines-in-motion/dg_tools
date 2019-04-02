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
  ,SetPosBiasSOUT( boost::bind(&ComImpedanceControl::set_pos_bias, this, _1,_2),
                positionSIN , "ComImpedanceControl("+name+")::output(vector)::set_pos_bias")
  ,SetVelBiasSOUT( boost::bind(&ComImpedanceControl::set_vel_bias, this, _1,_2),
                velocitySIN, "ComImpedanceControl("+name+")::output(vector)::set_vel_bias")
  ,isbiasset(0)
  ,init_flag(1)
  ,bias_time(10)
{
  init(TimeStep);
  Entity::signalRegistration(
    SetPosBiasSOUT << SetVelBiasSOUT << KpSIN << KdSIN << positionSIN << desiredpositionSIN <<
    velocitySIN << desiredvelocitySIN << feedforwardforceSIN <<  controlSOUT
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


    /*---------- computing position error ----*/
    pos_error.array() = des_pos.array() - position.array();
    vel_error.array() = des_vel.array() - velocity.array();

    // tau.array() = des_fff.array() + pos_error.array()*Kp.array()
    //               + vel_error.array()*Kd.array();

    tau.array() = pos_error.array();

    sotDEBUGOUT(15);

    return tau;

  }

dynamicgraph::Vector& ComImpedanceControl::
  set_pos_bias(dynamicgraph::Vector& pos_bias, int t){
 // TODO: ask Max about the reference (why do you need to return a ref?)

   const dynamicgraph::Vector& position = positionSIN(t);
   const dynamicgraph::Vector& velocity = velocitySIN(t);

    if(init_flag){
      t_start = t;
      pos_bias.resize(3); pos_bias.setZero();
      init_flag = 0;
      cout << "init_flag set" << endl;
      cout << t_start << endl;
      cout << init_flag << endl;
    }

    cout << "time" << t << endl;

    assert(pos_bias.size() == 3);

    if (isbiasset){
      cout << "bias has already been set" << endl;
      cout << pos_bias.array() << endl ;
    }
    else{
      if (t == t_start){
        cout << "assigning first element of pos_bias" << endl;
        pos_bias.array() = position.array()/bias_time;
        cout << "completed assigning first element of pos_bias" << endl;
      }
      else if(t < bias_time + t_start){
          cout << "setting mean bias" << endl;
          cout << t << endl;
          pos_bias.array() += position.array()/bias_time;
        }
      else {
        cout << "bias has already been set" << endl;

        isbiasset = 1;
      }
    }
    return pos_bias;
  }


dynamicgraph::Vector& ComImpedanceControl::
  set_vel_bias(dynamicgraph::Vector& vel_bias, int t){

    if(init_flag){
     t_start = t;
     vel_bias.setZero();
     init_flag = 0;
    }
    const dynamicgraph::Vector& position = positionSIN(t);
    const dynamicgraph::Vector& velocity = velocitySIN(t);
    if (isbiasset){
      // cout << "bias has already been set" << endl;

    }
    else{
      if (t == t_start){
        vel_bias.array() = velocity.array()/bias_time;
      }
      else if(t-t_start < bias_time){
          cout << t << endl;
          vel_bias.array() += velocity.array()/bias_time;
        }
      else {
        isbiasset = 1;
      }

    }
    return vel_bias;
}
