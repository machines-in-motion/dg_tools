/* This is a controller to achieve power jumps on the teststand */
//Author : Avadesh Meduri
// Date : 14/04/2019

#ifndef __SOT__power_jump_control_HH__
#define __SOT__power_jump_control_HH__

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/* SOT */
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/entity.h>

namespace dynamicgraph{
  namespace sot {
/* ---------------------------------------------------------------------------*/
/* ---- CLASS ----------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/

  class PowerJumpControl
    : public Entity
    {
    public:

      PowerJumpControl (const std::string &name);

    public:

      static const double TIME_STEP_DEFAULT;
      void init( const double& step){};


    public: /* ----- ENTITY INHERITANCE -----*/

      static const std::string CLASS_NAME;
      virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

    public: /* ------ SIGNALS ---------*/

      SignalPtr<dg::Vector, int> leg_lengthSIN; //current leg_length for trigger
      SignalPtr<dg::Vector, int> cnt_sensorSIN; // contact sensor value
      SignalPtr<dg::Vector, int> leg_length_triggerSIN; //position for trigger
      SignalPtr<dg::Vector, int> leg_length_airSIN; // desired position for landing
      SignalPtr<dg::Vector, int> des_fffSIN;
      SignalPtr<dg::Vector, int> des_weight_fffSIN;
      SignalPtr<dg::Vector, int> kp_groundSIN;
      SignalPtr<dg::Vector, int> kp_airSIN;

      // SignalPtr<dg::Vector, int> kdSIN;

      SignalTimeDependent<dg::Vector, int> return_des_posSOUT;
      SignalTimeDependent<dg::Vector, int> return_des_forceSOUT;
      SignalTimeDependent<dg::Vector, int> return_des_kpSOUT;

    protected:

      double TimeStep;
      dg::Vector& return_des_pos( dg::Vector &des_pos, int t);
      dg::Vector& return_des_force( dg::Vector &des_force, int t);
      dg::Vector& return_des_kp( dg::Vector &des_kp, int t);


      int pos_trigger_flag_;
      int force_trigger_flag_;
      int kp_trigger_flag_;


    };
  }
}

#endif
