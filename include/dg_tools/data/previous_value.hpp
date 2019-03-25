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
#  if defined (PreviousValue_EXPORTS)
#    define PreviousValue_EXPORTS __declspec(dllexport)
#  else
#    define PreviousValue_EXPORTS  __declspec(dllimport)
#  endif
#else
#  define PreviousValue_EXPORTS
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
  class PreviousValue_EXPORTS PreviousValue: public dg::Entity
  {
    public:

      PreviousValue( const std::string & name );

      void init(const int& size);

      static const std::string CLASS_NAME;
      virtual void display( std::ostream& os ) const;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      dg::SignalPtr<dg::Vector,int> dataSIN;
      dg::SignalTimeDependent<dg::Vector,int> dataSOUT;
      dg::SignalTimeDependent<dg::Vector,int> previousSOUT;

      dg::Vector& getPrevious(dg::Vector& previous, int time);
      dg::Vector& getInput(dg::Vector& output, int time);

    private:
      int size_;
      bool is_initialized_;
      dg::Vector internal_history_;
  };
} // namespace dg_tools
