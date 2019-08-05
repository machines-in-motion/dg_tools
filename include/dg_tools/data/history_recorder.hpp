/**
 * @file upsampler.hpp
 * @author Julian Viereck
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-08-05
 */

#pragma once

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/* SOT */
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/entity.h>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (HistoryRecorder_EXPORTS)
#    define HistoryRecorder_EXPORTS __declspec(dllexport)
#  else
#    define HistoryRecorder_EXPORTS  __declspec(dllimport)
#  endif
#else
#  define HistoryRecorder_EXPORTS
#endif

namespace dg_tools {

  /* --------------------------------------------------------------------- */
  /* --- CLASS ----------------------------------------------------------- */
  /* --------------------------------------------------------------------- */

  /**
   * @brief Records data over a fixed history and yields the data as truncated
   * vector. The newest entry is at the last position of the data array,
   *
   * history = [data_{t-n}, data_{t-n+1}, ..., data_{t-1}, data_{t}]
   *
   */
  class HistoryRecorder_EXPORTS HistoryRecorder: public dg::Entity
  {
    public:

      HistoryRecorder( const std::string & name );

      void init(const int& history_length);

      static const std::string CLASS_NAME;
      virtual void display( std::ostream& os ) const;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<dg::Vector,int> dataSIN;
      dg::SignalTimeDependent<dg::Vector,int> historySOUT;

      dg::Vector& getHistory(dg::Vector& history, int time);

    private:
      int history_length_;
      bool is_initialized_;
      dg::Matrix internal_history_;
  };
} // namespace dg_tools
