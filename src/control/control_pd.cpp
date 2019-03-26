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
#include "dg_tools/control/control_pd.hpp"

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ControlPD, "ControlPD");

const double ControlPD::
TIME_STEP_DEFAULT = .001;

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */


#define __SOT_ControlPD_INIT \

ControlPD::ControlPD( const std::string & name )
 :Entity(name)
 ,TimeStep(0)
 ,KpSIN(NULL,"ControlPD("+name+")::input(vector)::Kp")
 ,KdSIN(NULL,"ControlPD("+name+")::input(vector)::Kd")
 ,positionSIN(NULL,"ControlPD("+name+")::input(vector)::position")
 ,desiredpositionSIN(NULL,"ControlPD("+name+")::input(vector)::desired_position")
 ,velocitySIN(NULL,"ControlPD("+name+")::input(vector)::velocity")
 ,desiredvelocitySIN(NULL,"ControlPD("+name+")::input(vector)::desired_velocity")
 ,controlSOUT( boost::bind(&ControlPD::computeControl,this,_1,_2),
         KpSIN << KdSIN << positionSIN << desiredpositionSIN
         << velocitySIN << desiredvelocitySIN,
        "ControlPD("+name+")::output(vector)::control" )
  ,positionErrorSOUT(boost::bind(&ControlPD::computeControl,this,_1,_2),
        controlSOUT,
        "ControlPD("+name+")::output(vector)::position_error")
  ,velocityErrorSOUT(boost::bind(&ControlPD::computeControl,this,_1,_2),
        controlSOUT,
        "ControlPD("+name+")::output(vector)::velocity_error")
{
  Entity::signalRegistration( KpSIN << KdSIN << positionSIN <<
    desiredpositionSIN << velocitySIN << desiredvelocitySIN << controlSOUT
    << positionErrorSOUT << velocityErrorSOUT );
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void ControlPD::display( std::ostream& os ) const
{
  os << "ControlPD "<<getName();
  try{
    os <<"control = "<<controlSOUT;
  }
  catch (ExceptionSignal e) {}
  os <<" ("<<TimeStep<<") ";
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

double& ControlPD::setsize(int dimension)

{
  _dimension = dimension;
  return _dimension;
}

dynamicgraph::Vector& ControlPD::
computeControl( dynamicgraph::Vector &tau, int t )
{
  sotDEBUGIN(15);
  const dynamicgraph::Vector& Kp = KpSIN(t);
  const dynamicgraph::Vector& Kd = KdSIN(t);
  const dynamicgraph::Vector& position = positionSIN(t);
  const dynamicgraph::Vector& desired_position = desiredpositionSIN(t);
  const dynamicgraph::Vector& velocity = velocitySIN(t);
  const dynamicgraph::Vector& desired_velocity = desiredvelocitySIN(t);
      
  dynamicgraph::Vector::Index size = Kp.size();
  tau.resize(size);
  position_error.resize(size);
  velocity_error.resize(size);

  position_error.array() = desired_position.array()-position.array();
  velocity_error.array() = desired_velocity.array()-velocity.array();

  tau.array() = position_error.array()*Kp.array()
              + velocity_error.array()*Kd.array();

  sotDEBUGOUT(15);
  return tau;

}



dynamicgraph::Vector& ControlPD::
getPositionError( dynamicgraph::Vector &position_error, int t)
{
  //sotDEBUGOUT(15) ??
  controlSOUT(t);
  return position_error;
}

dynamicgraph::Vector& ControlPD::
getVelocityError( dynamicgraph::Vector &velocity_error, int t)
{
  controlSOUT(t);
  return velocity_error;
}
