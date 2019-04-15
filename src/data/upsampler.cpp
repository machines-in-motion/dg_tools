/**
 * Copyright 2019 Max Planck Society. All rights reserved.
 * Julian Viereck
 */

#include "dg_tools/data/upsampler.hpp"

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

using namespace dg_tools;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Upsampler, "Upsampler");

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

Upsampler::Upsampler( const std::string & name )
 :Entity(name)
 ,upsampling_factor_(1)
 ,is_initialized_(false)
 ,data_inputSIN(NULL,"Upsampler("+name+")::input(vector)::sin")
 ,data_outSOUT( boost::bind(&Upsampler::data_out_callback,this,_1,_2),
         data_inputSIN,
        "Upsampler("+name+")::output(vector)::sout" )
{
  Entity::signalRegistration(data_inputSIN << data_outSOUT);

  addCommand (
    "init",
    dynamicgraph::command::makeCommandVoid1(
      *this,
      &Upsampler::init,
      dynamicgraph::command::docCommandVoid1(
          "Init the upsampler",
          "int: upsampling factor (how often to repeat the previous input signal)")
    )
  );
}

void Upsampler::init(const int& upsampling_factor)
{
  upsampling_factor_ = upsampling_factor;
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

dg::Vector& Upsampler::data_out_callback(dg::Vector& out, int time)
{
  if (!is_initialized_) {
    is_initialized_ = true;
    first_time_ = time;
  }

  if ((time - first_time_) % upsampling_factor_ == 0) {
    // We use the time relative to the first time.
    last_input_ = data_inputSIN((time - first_time_) / upsampling_factor_);
  }

  out = last_input_;
  return out;
}
