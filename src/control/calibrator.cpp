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
 ,positionSIN(NULL,"Calibrator("+name+")::input(vector)::rawPosition")
 ,velocitySIN(NULL,"Calibrator("+name+")::input(vector)::velocity")
 ,desiredVelocitySIN(NULL,
              "Calibrator("+name+")::input(vector)::desiredVelocity")
 ,kpSIN(NULL,"Calibrator("+name+")::input(vector)::kp")
 ,hardstop2zeroSIN(NULL,"Calibrator("+name+")::input(vector)::hardstop2zero")
 ,positionSOUT( boost::bind(&Calibrator::compute_position,this,_1,_2),
         positionSIN << hardstop2zeroSIN,
        "Calibrator("+name+")::output(vector)::calibratedPosition" )
 ,controlSOUT( boost::bind(&Calibrator::calibrate,this,_1,_2),
 kpSIN << positionSIN << velocitySIN << desiredVelocitySIN,
 "Calibrator("+name+")::output(vector)::control" )
 ,initFlag(1)
//  calibratedFlagSOUT( boost::bind(&Calibrator::get_flag,this,_1,_2),
//  ,
//  "Calibrator("+name+")::output(vector)::control" )
{
  Entity::signalRegistration( positionSIN << velocitySIN << desiredVelocitySIN 
                  << kpSIN << hardstop2zeroSIN << positionSOUT << controlSOUT);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

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
dynamicgraph::Vector& Calibrator::calibrate( dynamicgraph::Vector &tau, int t ) 
{
  sotDEBUGIN(15);     
//   const dynamicgraph::Vector& position = positionSIN(t);
  
  const dynamicgraph::Vector& position = positionSIN(t);
  const dynamicgraph::Vector& velocity = velocitySIN(t);
  const dynamicgraph::Vector& desiredVelocity = desiredVelocitySIN(t);
  const dynamicgraph::Vector& kp = kpSIN(t);

  // TODO: tau should be a class member, the following should be in the init blk
  numJoints = position.size();
  tau.resize(numJoints); tau.setZero();
  if(initFlag){   
    
    tStart = t;
    error.resize(numJoints); error.setZero();
    isCalibrated.resize(numJoints); isCalibrated.setZero();
    start2hardstop.resize(numJoints); start2hardstop.setZero();

    initFlag = 0;
  }
  // hacky ramp. at 1 khz, this sould ramp up in half a second
  // TODO: use the initialized time
  error = desiredVelocity.array().min(
          desiredVelocity.array()/5000000.0*(t-tStart)).array() 
            - velocity.array();
  // error = desiredVelocity.array()-velocity.array();
  
  for(int idx = 0; idx < numJoints; idx++){
    if(!isCalibrated[idx]){
        tau[idx] = kp[idx]*error[idx];
    } else {
        tau[idx]=0; // turn motors off after calibration is finished
    }
  if(idx == 1){
    cout << "DEBUG: error of " << error[idx] << endl;
  }
    // check if we've saturated on error. If yes, save position and flip flag
    if(error[idx] >= 3.0*desiredVelocity[idx]){
      if(idx == 1){
        cout << "finished calibrating." << endl;
      }
        start2hardstop[idx] = position[idx];
        isCalibrated[idx] = true;
    }
  }

  // if all joints are calibrated, yay.
  
  sotDEBUGOUT(15);
  return tau;

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