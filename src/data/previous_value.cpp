/**
 * Copyright 2019 Max Planck Society. All rights reserved.
 * Julian Viereck
 */

#include "dg_tools/data/previous_value.hpp"

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

using namespace dg_tools;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PreviousValue, "PreviousValue");

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

PreviousValue::PreviousValue( const std::string & name )
 :Entity(name)
 ,size_(0)
 ,is_initialized_(false)
 ,dataSIN(NULL,"PreviousValue("+name+")::input(vector)::sin")
 ,dataSOUT( boost::bind(&PreviousValue::getInput,this,_1,_2),
        dataSIN,
        "PreviousValue("+name+")::output(vector)::sout" )
 ,previousSOUT( boost::bind(&PreviousValue::getPrevious,this,_1,_2),
        dataSIN,
        "PreviousValue("+name+")::output(vector)::sprev" )
{
  Entity::signalRegistration(dataSIN << dataSOUT << previousSOUT);

  addCommand (
    "init",
    dynamicgraph::command::makeCommandVoid1(
      *this,
      &PreviousValue::init,
      dynamicgraph::command::docCommandVoid1(
          "Init the entity",
          "int: Dimension of the stored value.")
    )
  );
}

void PreviousValue::init(const int& size)
{
  internal_history_.resize(size);
  internal_history_.fill(0.);
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void PreviousValue::display( std::ostream& os ) const
{
  os << "PreviousValue " << getName();
  os <<" (" << internal_history_.size() << ") ";
}

dg::Vector& PreviousValue::getInput(dg::Vector& output, int time)
{
  output = dataSIN(time);
  assert(output.size() == internal_history_.size());
  internal_history_ = output;
  return output;
}

dg::Vector& PreviousValue::getPrevious(dg::Vector& previous, int time)
{
  previous = internal_history_;
  return internal_history_;
}
