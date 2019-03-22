/*
 * Copyright 2010,
 * François Bleibel,
 * Olivier Stasse,
 *
 * CNRS/AIST
 *
 * This file is part of sot-core.
 * sot-core is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-core is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-core.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SOT_Calibrator_HH__
#define __SOT_Calibrator_HH__

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
#  if defined (calibrator_EXPORTS)
#    define Calibrator_EXPORT __declspec(dllexport)
#  else  
#    define Calibrator_EXPORT  __declspec(dllimport)
#  endif 
#else
#  define Calibrator_EXPORT
#endif

namespace dynamicgraph {
  namespace sot {

  /* --------------------------------------------------------------------- */
  /* --- CLASS ----------------------------------------------------------- */
  /* --------------------------------------------------------------------- */

  class Calibrator_EXPORT Calibrator
    : public Entity
    {

    public: /* --- CONSTRUCTOR ---- */

      Calibrator( const std::string & name );

    public: /* --- INIT --- */

      void init( const double& step);

    public: /* --- CONSTANTS --- */

      /* Default values. */
      static const double TIME_STEP_DEFAULT;   // = 0.001

    public: /* --- ENTITY INHERITANCE --- */
      static const std::string CLASS_NAME;
      virtual void display( std::ostream& os ) const; 
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}


    protected: 
  
      /* Parameters of the torque-control function: 
       * tau = kp * (qd-q) + kd* (dqd-dq) */
      double TimeStep;
      double _dimension;

    public:  /* --- SIGNALS --- */
      SignalPtr<dg::Vector, int> positionSIN;
      SignalPtr<dg::Vector, int> velocitySIN;
      SignalPtr<dg::Vector, int> desiredVelocitySIN;
      SignalPtr<dg::Vector,int> kpSIN;
      // this should be the offset that brings your legs to 0 configuration
      SignalPtr<dg::Vector, int> hardstop2zeroSIN;
      

      // OUTPUT SIGNALS
      SignalTimeDependent<dg::Vector, int> positionSOUT;
      // Only used during calibration part
      SignalTimeDependent<dg::Vector,int> controlSOUT;

    protected:

      double& setsize(int dimension);
      int tStart;
      int initFlag;
      int numJoints;
      dg::Vector& calibrate( dg::Vector& tau, int t );
      dg::Vector& compute_position( dg::Vector& pos, int t);
      // dg::Bool& get_flag( dg::Bool& flag, int t);
      
      // this is to keep track of when we start
      // we do not check the threshold in the first X seconds,
      // otherwise we could get a false positive. Hardcoded.
      dg::Vector isCalibrated;
      dg::Vector desVel;
      dg::Vector error;
      dg::Vector start2hardstop; // this is recorded during calibration
      // dg::Vector& getPositionError( dg::Vector& position_error,int t );
      // dg::Vector& getVelocityError( dg::Vector& velocity_error,int t );

    };



} // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __SOT_Calibrator_HH__
