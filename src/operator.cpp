/**
 * Copyright 2019 Max Planck Society. All rights reserved.
 * Julian Viereck
 */

#include "dg_tools/operator.hpp"

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

#include <iostream>

using namespace dg_tools;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PoseQuaternionToPoseRPY, "PoseQuaternionToPoseRPY");

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

PoseQuaternionToPoseRPY::PoseQuaternionToPoseRPY( const std::string & name )
 :Entity(name)
 ,data_inputSIN(NULL,"PoseQuaternionToPoseRPY("+name+")::input(vector)::sin")
 ,data_outSOUT( boost::bind(&PoseQuaternionToPoseRPY::data_out_callback,this,_1,_2),
         data_inputSIN,
        "PoseQuaternionToPoseRPY("+name+")::output(vector)::sout" )
{
  Entity::signalRegistration(data_inputSIN << data_outSOUT);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

dg::Vector& PoseQuaternionToPoseRPY::data_out_callback(dg::Vector& out, int time)
{
    const dg::Vector& input = data_inputSIN(time);
    dynamicgraph::sot::VectorQuaternion quat(input(6), input(3), input(4), input(5));

    out.resize(6);
    out.head<3>() = input.head<3>();
    out.tail<3>() = quat.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
    return out;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SinEntity, "Sin");

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

SinEntity::SinEntity( const std::string & name )
 :Entity(name)
 ,data_inputSIN(NULL,"Sine("+name+")::input(double)::sin")
 ,data_outSOUT( boost::bind(&SinEntity::data_out_callback,this,_1,_2),
         data_inputSIN,
        "SinEntity("+name+")::output(double)::sout" )
{
  Entity::signalRegistration(data_inputSIN << data_outSOUT);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

double& SinEntity::data_out_callback(double& out, int time)
{
    out = sin(data_inputSIN.access(time));
    return out;
}


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Division_of_double, "Division_of_double");

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

Division_of_double::Division_of_double( const std::string & name )
 :Entity(name)
 ,data_input1SIN(NULL,"Division_of_double("+name+")::input(double)::sin1")
 ,data_input2SIN(NULL,"Division_of_double("+name+")::input(double)::sin2")
 ,data_outSOUT( boost::bind(&Division_of_double::data_out_callback,this,_1,_2),
         data_input1SIN << data_input2SIN,
        "Division_of_double("+name+")::output(double)::sout" )
{
  Entity::signalRegistration(data_input1SIN << data_input2SIN << data_outSOUT);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

double& Division_of_double::data_out_callback(double& out, int time)
{
    out = data_input1SIN.access(time) / data_input2SIN.access(time);
    return out;
}



DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(VectorIntegrator, "VectorIntegrator");

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

VectorIntegrator::VectorIntegrator( const std::string & name )
 :Entity(name)
 ,init_(true)
 ,data_inputSIN(NULL,"VectorIntegrator("+name+")::input(vector)::sin")
 ,data_outSOUT( boost::bind(&VectorIntegrator::data_out_callback,this,_1,_2),
         data_inputSIN,
        "VectorIntegrator("+name+")::output(vector)::sout" )
{
  Entity::signalRegistration(data_inputSIN << data_outSOUT);

  data_outSOUT.setDependencyType(
        dynamicgraph::TimeDependency<int>::ALWAYS_READY);
}

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

dg::Vector& VectorIntegrator::data_out_callback(dg::Vector& out, int time)
{
  const dg::Vector& input = data_inputSIN(time);
  out.resize(input.size());
  if (init_) {
    init_ = false;
    sum_.resize(input.size());
    sum_.setZero(input.size());
  }

  // TODO: Avoid hard-coded sampling rate here.
  sum_ += 0.001 * input;
  out = sum_;
  return out;
}
