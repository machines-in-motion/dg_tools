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
#  if defined (MemoryReplay_EXPORTS)
#    define MemoryReplay_EXPORTS __declspec(dllexport)
#  else
#    define MemoryReplay_EXPORTS  __declspec(dllimport)
#  endif
#else
#  define MemoryReplay_EXPORTS
#endif

namespace dg_tools {

  /* --------------------------------------------------------------------- */
  /* --- CLASS ----------------------------------------------------------- */
  /* --------------------------------------------------------------------- */

  /**
   * @brief Provided with a matrix, the entity returns the row indexed by
   * the current time index at every timestep.
   *
   * Example: Store a desired trajectory in memory and get the desired position
   * at every timestep.
   */
  class MemoryReplay_EXPORTS MemoryReplay: public dg::Entity
  {
    public:

      MemoryReplay( const std::string & name );

      void init(const dg::Matrix& data);
      void rewind();

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalTimeDependent<dg::Vector,int> internal_signal_refresher_;
      dg::SignalTimeDependent<dg::Vector,int> sout;

      dg::Vector& getValue(dg::Vector& sout, int time);

    private:
      bool is_started_;
      int start_time_;
      dg::Matrix data_;
  };
} // namespace dg_tools
