/**
 * Copyright 2019 Max Planck Society. All rights reserved.
 * Julian Viereck
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
#  if defined (Upsampler_EXPORTS)
#    define Upsampler_EXPORTS __declspec(dllexport)
#  else
#    define Upsampler_EXPORTS  __declspec(dllimport)
#  endif
#else
#  define Upsampler_EXPORTS
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
  class Upsampler_EXPORTS Upsampler: public dg::Entity
  {
    public:

      Upsampler( const std::string & name );

      void init(const int& upsampling_factor);

      static const std::string CLASS_NAME;
      virtual void display( std::ostream& os ) const;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<dg::Vector,int> data_inputSIN;
      dg::SignalTimeDependent<dg::Vector,int> data_outSOUT;

      dg::Vector& data_out_callback(dg::Vector& history, int time);
    private:
      bool is_initialized_;
      int upsampling_factor_;
      int first_time_;
      dg::Vector last_input_;
  };
} // namespace dg_tools
