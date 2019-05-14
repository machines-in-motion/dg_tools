/* This is a controller to achieve power jumps on the teststand */

#include <iostream>
#include "dg_tools/test_stand_control/power_jump.hpp"

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
using namespace std;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PowerJumpControl, "power_jump_control");

PowerJumpControl::PowerJumpControl(const std::string &name)
  :Entity(name)
  ,TimeStep(0)
  ,leg_lengthSIN(NULL, "PowerJumpControl("+name+"):input(vector)::leg_length")
  ,cnt_sensorSIN(NULL, "PowerJumpControl("+name+"):input(vector)::cnt_sensor")
  ,leg_length_triggerSIN(NULL, "PowerJumpControl("+name+"):input(vector)::leg_length_trigger")
  ,leg_length_airSIN(NULL, "PowerJumpControl("+name+"):input(vector)::leg_length_air")
  ,des_fffSIN(NULL, "PowerJumpControl("+name+"):input(vector)::des_fff")
  ,des_weight_fffSIN(NULL, "PowerJumpControl("+name+"):input(vector)::des_weight_fff")
  ,kp_groundSIN(NULL, "PowerJumpControl("+name+"):input(vector)::kp_ground")
  ,kp_airSIN(NULL, "PowerJumpControl("+name+"):input(vector)::kp_air")


  ,return_des_posSOUT ( boost::bind(&PowerJumpControl::return_des_pos, this, _1, _2),
                         leg_lengthSIN << cnt_sensorSIN << leg_length_airSIN <<
                         leg_length_triggerSIN,
                         "PowerJumpControl("+name+")::output(vector)::des_pos")
 ,return_des_forceSOUT ( boost::bind(&PowerJumpControl::return_des_force, this, _1, _2),
                        leg_lengthSIN << cnt_sensorSIN << des_fffSIN <<
                        des_weight_fffSIN,
                        "PowerJumpControl("+name+")::output(vector)::des_force")
 ,return_des_kpSOUT ( boost::bind(&PowerJumpControl::return_des_kp, this, _1, _2),
                       leg_lengthSIN << cnt_sensorSIN << des_fffSIN <<
                       des_weight_fffSIN,
                       "PowerJumpControl("+name+")::output(vector)::des_kp")
  ,pos_trigger_flag_(1)
  ,force_trigger_flag_(1)
  ,kp_trigger_flag_(1)

  {
    init(TimeStep);
    Entity::signalRegistration(
      leg_lengthSIN << cnt_sensorSIN << leg_length_airSIN << leg_length_triggerSIN
      << des_fffSIN << des_weight_fffSIN << return_des_posSOUT << return_des_forceSOUT
      << kp_groundSIN << kp_airSIN << return_des_kpSOUT
    );
  }


dynamicgraph::Vector& PowerJumpControl::
  return_des_pos( dynamicgraph::Vector &des_pos, int t){
    sotDEBUGIN(15);

    /** This method computes desired position depending on state of
    the teststand **/

    const dynamicgraph::Vector& cnt_sensor = cnt_sensorSIN(t);
    const dynamicgraph::Vector& leg_length = leg_lengthSIN(t);
    const dynamicgraph::Vector& leg_length_trigger = leg_length_triggerSIN(t);
    const dynamicgraph::Vector& leg_length_air = leg_length_airSIN(t);

    des_pos.resize(6);

    if (cnt_sensor[0] < .2){
      if (pos_trigger_flag_){
        if (leg_length[0] > leg_length_trigger[0]){
          // going to desired position before pushing of the ground
          des_pos.array() = leg_length_trigger.array();
        }
        else if(leg_length[0] == leg_length_trigger[0]){
          pos_trigger_flag_ = 0;
          des_pos.array() = leg_length_trigger.array();
        }
      }
      else if (!pos_trigger_flag_){
        des_pos.array() = leg_length.array();
      }
    }

    else{
      des_pos.array() = leg_length_air.array();
      pos_trigger_flag_ = 1;
    }

    sotDEBUGOUT(15);
    return des_pos;
  }

dynamicgraph::Vector& PowerJumpControl::
  return_des_force( dynamicgraph::Vector &des_force, int t){
    sotDEBUGIN(15);

    /** This method computes desired forces depending on state of
    the teststand **/
    const dynamicgraph::Vector& cnt_sensor = cnt_sensorSIN(t);
    const dynamicgraph::Vector& leg_length = leg_lengthSIN(t);
    const dynamicgraph::Vector& leg_length_trigger = leg_length_triggerSIN(t);
    const dynamicgraph::Vector& des_fff = des_fffSIN(t);
    const dynamicgraph::Vector& des_weight_fff = des_weight_fffSIN(t);

    des_force.resize(6);

    if (cnt_sensor[0] < 0.2){
      if (force_trigger_flag_){
        if (leg_length[0] > leg_length_trigger[0]){
          // going to desired position before pushing of the ground
          des_force.array() = des_weight_fff;
        }
        else if(leg_length[0] == leg_length_trigger[0]){
          force_trigger_flag_ = 0;
          des_force.array() = des_weight_fff;
        }
      }
      else if (!force_trigger_flag_){
        des_force.array() = des_fff;
      }
    }

    else{
      des_force[0] = 0.0;
      des_force[1] = 0.0;
      des_force[2] = 0.0;
      des_force[3] = 0.0;
      des_force[4] = 0.0;
      des_force[5] = 0.0;
    }

    sotDEBUGOUT(15);
    return des_force;
  }

dynamicgraph::Vector& PowerJumpControl::
  return_des_kp( dg::Vector &des_kp, int t){
    sotDEBUGIN(15);

    /** This method computes desired kp gain depending on state of
    the teststand **/
    const dynamicgraph::Vector& cnt_sensor = cnt_sensorSIN(t);
    const dynamicgraph::Vector& leg_length = leg_lengthSIN(t);
    const dynamicgraph::Vector& leg_length_trigger = leg_length_triggerSIN(t);
    const dynamicgraph::Vector& kp_ground = kp_groundSIN(t);
    const dynamicgraph::Vector& kp_air = kp_airSIN(t);

    des_kp.resize(6);

    if (cnt_sensor[0] < 0.2){
      if (kp_trigger_flag_){
        if (leg_length[0] > leg_length_trigger[0]){
          // going to desired position before pushing of the ground
          des_kp.array() = kp_ground.array();
        }
        else if(leg_length[0] == leg_length_trigger[0]){
          kp_trigger_flag_ = 0;
          des_kp.array() = kp_ground.array();
        }
      }
      else if (!kp_trigger_flag_){
        des_kp[0] = 0.0;
        des_kp[1] = 0.0;
        des_kp[2] = 0.0;
        des_kp[3] = 0.0;
        des_kp[4] = 0.0;
        des_kp[5] = 0.0;
      }
    }

    else{
      des_kp.array() = kp_air.array();
    }

    sotDEBUGOUT(15);
    return des_kp;
  }
