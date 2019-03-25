/**
 * Copyright 2019 Max Planck Society. All rights reserved.
 * Julian Viereck
 */

#include "dg_tools/data/memory_replay.hpp"

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-commands.h>

#include <algorithm>

using namespace dg_tools;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(MemoryReplay, "MemoryReplay");

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

MemoryReplay::MemoryReplay( const std::string & name )
 :Entity(name)
 ,is_started_(false)
 ,internal_signal_refresher_(
    "MemoryReplay("+name+")::intern(dummy)::refresher" )
 ,sout( boost::bind(&MemoryReplay::getValue,this,_1,_2),
    internal_signal_refresher_,
    "MemoryReplay("+name+")::output(vector)::sout" )
{
  data_.resize(0, 0);

  // Define the refresh signal as always ready.
  internal_signal_refresher_.setDependencyType(
    dynamicgraph::TimeDependency<int>::ALWAYS_READY);


  Entity::signalRegistration(sout);

  addCommand (
    "init",
    dynamicgraph::command::makeCommandVoid1(
      *this,
      &MemoryReplay::init,
      dynamicgraph::command::docCommandVoid1("Init and set the memory to replay.",
        "Matrix: Memory to reply. The rows are indexed by time.")
    )
  );

  addCommand (
    "rewind",
    dynamicgraph::command::makeCommandVoid0(
      *this,
      &MemoryReplay::rewind,
      dynamicgraph::command::docCommandVoid0("Rewinds the data to the first position again.")
    )
  );
}

void MemoryReplay::init(const dg::Matrix& data)
{
  data_ = data;
}

void MemoryReplay::rewind()
{
  is_started_ = false;
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

dg::Vector& MemoryReplay::getValue(dg::Vector& sout, int time)
{
  assert(data_.rows() > 0);

  if (!is_started_) {
    start_time_ = time;
    is_started_ = true;
  }

  // Return either the row at the time index or the last row.
  sout = data_.row(std::min(time - start_time_, (int)(data_.rows() - 1)));
  return sout;
}

