/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

/* SOT */
#include "dg_tools/control/calibrator.hpp"

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

#include <iostream>
using namespace std;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Calibrator, "Calibrator");

const double Calibrator::
TIME_STEP_DEFAULT = .001;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#define __SOT_Calibrator_INIT \

Calibrator::Calibrator( const std::string & name )
 :Entity(name)
 ,positionSIN(NULL,"Calibrator("+name+")::input(vector)::raw_position")
 ,velocitySIN(NULL,"Calibrator("+name+")::input(vector)::velocity")
 ,calibration_torqueSIN(NULL,
              "Calibrator("+name+")::input(vector)::calibration_torque")
//  ,kpSIN(NULL,"Calibrator("+name+")::input(vector)::kp")
 ,hardstop2zeroSIN(NULL,"Calibrator("+name+")::input(vector)::hardstop2zero")
 ,positionSOUT( boost::bind(&Calibrator::compute_position,this,_1,_2),
         positionSIN << hardstop2zeroSIN,
        "Calibrator("+name+")::output(vector)::calibrated_position" )
 ,controlSOUT( boost::bind(&Calibrator::calibrate,this,_1,_2),
  positionSIN << velocitySIN << calibration_torqueSIN,
 "Calibrator("+name+")::output(vector)::control" )
 ,init_flag(1)
 ,calibrated_flag(0)
 ,threshold_time(1000) // threshold_time used to ramp up
 ,threshold_velocity(0.001)
//  calibratedFlagSOUT( boost::bind(&Calibrator::get_flag,this,_1,_2),
//  ,
//  "Calibrator("+name+")::output(vector)::control" )
{
  Entity::signalRegistration( positionSIN << velocitySIN << hardstop2zeroSIN << 
                          calibration_torqueSIN << positionSOUT << controlSOUT);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

// TODO: setters for thresholds

void Calibrator::display( std::ostream& os ) const
{
  os << "Calibrator "<<getName();
  try{
    os <<"control = "<<controlSOUT; 
  }
  catch (ExceptionSignal e) {}
  os <<" ("<<TimeStep<<") ";
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

double& Calibrator::setsize(int dimension)
{
  _dimension = dimension;
  return _dimension;
}

// velocity control in a direction or other
dynamicgraph::Vector& 
    Calibrator::calibrate( dynamicgraph::Vector &torque, int t ) 
{
  sotDEBUGIN(15);     
//   const dynamicgraph::Vector& position = positionSIN(t);
  
  const dynamicgraph::Vector& position = positionSIN(t);
  const dynamicgraph::Vector& velocity = velocitySIN(t);
  const dynamicgraph::Vector& calibration_torque = calibration_torqueSIN(t);
  // const dynamicgraph::Vector& kp = kpSIN(t);

  // TODO: torque should be a class member, 
  // the following should be in the init blk
  num_joints = position.size();
  torque.resize(num_joints); torque.setZero();
  if(init_flag){   
    cout << "Begin calibration" << endl;
    t_start = t;
    error.resize(num_joints); error.setZero();
    is_calibrated.resize(num_joints); is_calibrated.setZero();
    start2hardstop.resize(num_joints); start2hardstop.setZero();
    // maxTau.resize(num_joints); maxTau.setOnes();

    init_flag = 0;
  }
  // ramp to calibration torque in threshold_time, then hold
  // TODO: use the initialized time
  torque = calibration_torque.array().min(
          calibration_torque.array()/threshold_time*(t-t_start)).array();

  for(int idx = 0; idx < num_joints; idx++){
    if(is_calibrated[idx]){
        torque[idx]=0; // turn motors off after calibration is finished
    }
    if(t%100 == 0 && idx ==1){
      cout << " error and vel at: e(" << error[idx]
              <<") > v("<< velocity[idx] << ")" << endl;
      cout << endl;    
    }
    // check if we've saturated on error. If yes, save position and flip flag
    // TODO: get rid of magic number
    if((t-t_start)>threshold_time && abs(velocity[idx])<threshold_velocity){
      start2hardstop[idx] = position[idx];
      if(t%100 == 0 && idx ==1){
        cout << "threshold triggered at: e(" << error[idx]
                <<") > v("<< velocity[idx] << ")" << endl;
        cout << endl;
      }
      is_calibrated[idx] = true;
    }
  }

  // if all joints are calibrated, yay.
  if(is_calibrated.sum() == num_joints && !calibrated_flag){
    cout << "*** Calibration completed ***" << endl;
    calibrated_flag = 1;
  }
  // torque = torque.array().min(maxTau.array());
  sotDEBUGOUT(15);
  return torque;
}

dynamicgraph::Vector &Calibrator::
compute_position(dynamicgraph::Vector &calibratedPosition, int t)
{
    sotDEBUGIN(15);

    const dynamicgraph::Vector &position = positionSIN(t);
    const dynamicgraph::Vector &hardstop2zero = hardstop2zeroSIN(t);

    calibratedPosition.resize(position.size());
    if(start2hardstop.size() == 0){ // calibration has not yet happened!
      start2hardstop.resize(position.size());
      start2hardstop.setZero();
    }

    calibratedPosition.array() = position.array() - start2hardstop.array()
                                 + hardstop2zero.array();

    sotDEBUGOUT(15);
    return calibratedPosition;
}