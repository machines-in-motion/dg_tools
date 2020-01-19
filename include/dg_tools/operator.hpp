/**
 * @file VectorToQuaternion.hpp
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

#include <sot/core/matrix-geometry.hh>


/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (Operator_EXPORTS)
#    define Operator_EXPORTS __declspec(dllexport)
#  else
#    define Operator_EXPORTS  __declspec(dllimport)
#  endif
#else
#  define Operator_EXPORTS
#endif

namespace dg_tools {

  /* --------------------------------------------------------------------- */
  /* --- CLASS ----------------------------------------------------------- */
  /* --------------------------------------------------------------------- */

  /**
   * @brief Upsamples the input data by holding the last value for a number of
   * timesteps.
   *
   * Pulls the input signal only when the time signal is a multiplicative
   * of the sampling factor.
   */
  class Operator_EXPORTS PoseQuaternionToPoseRPY: public dg::Entity
  {
    public:

      PoseQuaternionToPoseRPY( const std::string & name );

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<dg::Vector,int> data_inputSIN;
      dg::SignalTimeDependent<dg::Vector,int> data_outSOUT;

      dg::Vector& data_out_callback(dg::Vector& history, int time);
  };
} // namespace dg_tools
