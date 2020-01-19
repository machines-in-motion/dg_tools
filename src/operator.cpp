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
    dynamicgraph::sot::VectorQuaternion quat;
    quat.coeffs() = input.tail<4>();

    out.resize(6);
    out.head<3>() = input.head<3>();
    out.tail<3>() = quat.toRotationMatrix().eulerAngles(0, 1, 2);
    return out;
}
