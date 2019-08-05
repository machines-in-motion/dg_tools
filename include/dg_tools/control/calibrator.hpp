/**
 * @file Steve Heim
 * @author Julian Viereck
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-08-05
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

    public: /* --- CONSTANTS --- */

      /* Default values. */
      static const double TIME_STEP_DEFAULT;   // = 0.001

    public: /* --- ENTITY INHERITANCE --- */
      static const std::string CLASS_NAME;
      virtual void display( std::ostream& os ) const; 
      virtual const std::string& getClassName( void ) const 
              {return CLASS_NAME;}


    protected: 
  
      double TimeStep;
      double _dimension;

    public:  /* --- SIGNALS --- */
      SignalPtr<dg::Vector, int> positionSIN;
      SignalPtr<dg::Vector, int> velocitySIN;
      SignalPtr<dg::Vector, int> calibration_torqueSIN;
      // this should be the offset that brings your legs to 0 configuration
      SignalPtr<dg::Vector, int> hardstop2zeroSIN;
      

      // OUTPUT SIGNALS
      SignalTimeDependent<dg::Vector, int> positionSOUT;
      // Only used during calibration part
      SignalTimeDependent<dg::Vector, int> controlSOUT;
      SignalTimeDependent<int, int> calibrated_flagSOUT;

    protected:

      // signal refresher can be used to trigger SOUTs that do not depend on
      // any SINs.
      SignalTimeDependent<int, int> internal_signal_refresher_;

      double& setsize(int dimension);
      int threshold_time; // in timesteps
      double threshold_velocity;
      int t_start;
      int init_flag;
      int num_joints;
      dg::Vector& calibrate( dg::Vector& tau, int t );
      dg::Vector& compute_position( dg::Vector& pos, int t);
      int& is_calibrated( int& calibrated_flag, int t);
      int calibrated_flag_;
      dg::Vector calibrated;
      dg::Vector des_vel;
      dg::Vector error;
      dg::Vector start2hardstop; // this is recorded during calibration

    };



} // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __SOT_Calibrator_HH__
