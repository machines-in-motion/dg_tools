/**
 * Copyright 2019 Max Planck Society. All rights reserved.
 * Julian Viereck
 */

#include "dg_tools/data/history_recorder.hpp"

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace dg_tools;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(HistoryRecorder, "HistoryRecorder");

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

HistoryRecorder::HistoryRecorder( const std::string & name )
 :Entity(name)
 ,history_length_(0)
 ,is_initialized_(false)
 ,dataSIN(NULL,"HistoryRecorder("+name+")::input(vector)::sin")
 ,historySOUT( boost::bind(&HistoryRecorder::getHistory,this,_1,_2),
         dataSIN,
        "HistoryRecorder("+name+")::output(vector)::sout" )
{
  Entity::signalRegistration(dataSIN << historySOUT);
}

void HistoryRecorder::init(const int& history_length)
{
  history_length_ = history_length;
}


/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

dg::Vector& HistoryRecorder::getHistory( dg::Vector& history, int time)
{
  const dg::Vector& input = dataSIN(time);

  if (!is_initialized_) {
    is_initialized_ = true;

    // Allocate the result vector based on the input size.
    internal_history_.resize(history_length_, input.size());

    // Fill the history with the first input data.
    for (int i = 0; i < history_length_; i++) {
      internal_history_.col(i) = input;
    }
  }

  // Move the data in the matrix to the front.
  for (int i = 1; i < history_length_; i++) {
    internal_history_.col(i - 1) = internal_history_.col(i);
  }

  // Put the new data at the end of the internal history.
  internal_history_.rightCols(1) = input;

  // The new Eigen::Matrix.reshaped API is not available in the Eigen version
  // we use.
  Eigen::Map<dg::Vector> interval_history_vec(
      internal_history_.data(), internal_history_.size());

  // Return the linearized reshaped version of the history.
  history = interval_history_vec;
  return history;
}
