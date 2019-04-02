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
  ,biasedpositionSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::biased_pos")
  ,velocitySIN(NULL, "ComImpedanceControl("+name+")::input(vector)::velocity")
  ,desiredvelocitySIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_vel")
  ,biasedvelocitySIN(NULL, "ComImpedanceControl("+name+")::input(vector)::biased_vel")
  ,feedforwardforceSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_fff")
  ,inertiaSIN(NULL, "ComImpedanceControl("+name+")::input(vector::inertia)")
  ,angvelSIN(NULL, "ComImpedanceControl("+name+")::input(vector::angvel)")
  ,desiredangvelSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_angvel")
  ,feedforwardtorquesSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_fft")
  ,controlSOUT( boost::bind(&ComImpedanceControl::return_control_torques, this, _1,_2),
                KpSIN << KdSIN << positionSIN << desiredpositionSIN <<
                velocitySIN << desiredvelocitySIN << feedforwardforceSIN ,
                "ComImpedanceControl("+name+")::output(vector)::tau")
  ,angcontrolSOUT( boost::bind(&ComImpedanceControl::return_angcontrol_torques, this, _1,_2),
                  KpSIN << KdSIN << inertiaSIN <<angvelSIN << desiredangvelSIN
                  << feedforwardtorquesSIN,
                    "ComImpedanceControl("+name+")::output(vector)::angtau")
  ,SetPosBiasSOUT( boost::bind(&ComImpedanceControl::set_pos_bias, this, _1,_2),
                positionSIN , "ComImpedanceControl("+name+")::output(vector)::set_pos_bias")
  ,SetVelBiasSOUT( boost::bind(&ComImpedanceControl::set_vel_bias, this, _1,_2),
                velocitySIN, "ComImpedanceControl("+name+")::output(vector)::set_vel_bias")
  ,isbiasset(0)
  ,init_flag(1)
  ,bias_time(100)
{
  init(TimeStep);
  Entity::signalRegistration(
    positionSIN << velocitySIN << SetPosBiasSOUT << SetVelBiasSOUT << KpSIN << KdSIN <<
    biasedpositionSIN << biasedvelocitySIN << desiredpositionSIN <<
    desiredvelocitySIN << feedforwardforceSIN <<  controlSOUT << angcontrolSOUT
  );
}

dynamicgraph::Vector& ComImpedanceControl::
  return_control_torques( dynamicgraph::Vector &tau, int t){
    sotDEBUGIN(15);
    const dynamicgraph::Vector& Kp = KpSIN(t);
    const dynamicgraph::Vector& Kd = KdSIN(t);
    const dynamicgraph::Vector& position = biasedpositionSIN(t);
    const dynamicgraph::Vector& des_pos = desiredpositionSIN(t);
    const dynamicgraph::Vector& velocity = biasedvelocitySIN(t);
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


    // /*---------- bias values ----*/
    // if (isbiasset){
    //   pos_error.array() += position_bias.array();
    //   vel_error.array() += velocity_bias.array();
    // }

    if (isbiasset){

    /*---------- computing position error ----*/
    pos_error.array() = des_pos.array() - position.array();
    vel_error.array() = des_vel.array() - velocity.array();

    /*---------- computing tourques ----*/

    tau.array() = des_fff.array() + pos_error.array()*Kp.array()
                  + vel_error.array()*Kd.array();

    }
    else{
      // quick hack to return zeros
      tau.array() = des_pos.array() - des_pos.array();
    }

    sotDEBUGOUT(15);

    return tau;

  }

dynamicgraph::Vector& ComImpedanceControl::
  return_angcontrol_torques( dynamicgraph::Vector &angtau, int t){
    sotDEBUGIN(15);

    const dynamicgraph::Vector& Kp = KpSIN(t);
    const dynamicgraph::Vector& Kd = KdSIN(t);
    const dynamicgraph::Vector& inertia = inertiaSIN(t);
    const dynamicgraph::Vector& omega = angvelSIN(t);
    const dynamicgraph::Vector& omega_des = desiredangvelSIN(t);
    const dynamicgraph::Vector& hd_des = feedforwardtorquesSIN(t);

    /*----- assertions of sizes -------------*/
    assert(Kp.size() == 3);
    assert(Kd.size() == 3);
    assert(omega.size() == 3);
    assert(omega_des.size() == 3);
    assert(hd_des.size()==3);

    /*---------- computing ang error ----*/
    h_error.array() = inertia.array()*(omega.array() - omega_des.array());

    angtau.array() = hd_des.array() + Kp.array() * h_error.array();


    sotDEBUGOUT(15);
    return angtau;
  }



dynamicgraph::Vector& ComImpedanceControl::
  set_pos_bias(dynamicgraph::Vector& pos_bias, int t){
    // NOTE : when a value of the signal is not computed at all timesteps
    // create another variable tp compute the value and assign it to the signal
    // at all time steps to ensure that the value of the signal is constant.
    // When not done, the signal value fluctuates

    sotDEBUGIN(15);
    const dynamicgraph::Vector& position = positionSIN(t);

    position_bias.resize(3);
    pos_bias.resize(3);

    if(init_flag){
      t_start = t;
      position_bias.setZero();
      pos_bias.setZero();
      init_flag = 0;
    }

    if (isbiasset){
      // cout << "bias set" << endl;
      for(int j = 0; j < 3 ; j++){
        pos_bias[j] = position_bias[j];
      }
    }
    else{
      for(int i = 0 ; i < 3; i ++){
        if(t < bias_time + t_start){
            position_bias[i] += position[i]/bias_time;
          }
        else {
          isbiasset = 1;
        }
        pos_bias[i] = position_bias[i];
      }
    }

    sotDEBUGOUT(15);
    return pos_bias;

  }


dynamicgraph::Vector& ComImpedanceControl::
  set_vel_bias(dynamicgraph::Vector& vel_bias, int t){
    sotDEBUGIN(15);
    const dynamicgraph::Vector& velocity = velocitySIN(t);

    velocity_bias.resize(3);
    vel_bias.resize(3);

    if(init_flag){
      t_start = t;
      velocity_bias.setZero();
      vel_bias.setZero();
      init_flag = 0;
    }

    if (isbiasset){
      // cout << "bias set" << endl;
      for(int j = 0; j < 3 ; j++){
        vel_bias[j] = velocity_bias[j];
      }
    }
    else{
      for(int i = 0 ; i < 3; i ++){
        if(t < bias_time + t_start){
            velocity_bias[i] += velocity[i]/bias_time;
          }
        else {
          isbiasset = 1;
        }
        vel_bias[i] = velocity_bias[i];
      }
    }

    sotDEBUGOUT(15);
    return vel_bias;

  }
