// impedance controller implementation for COM (used for quadruped)
// Author : Avadesh Meduri
// Date : 25/03/19


#ifndef __SOT_com_impedance_Control_HH__
#define  __SOT_com_impedance_Control_HH__


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

    class ComImpedanceControl
      : public Entity
      {
      public: /*------ Constructor --------*/

        ComImpedanceControl( const std::string &name);

      public: /*------ init --------*/

        void init( const double& step){};

      public: /*------ CONSTANTS --------*/

        static const double TIME_STEP_DEFAULT;

      public: /* ----- ENTITY INHERITANCE -----*/

        static const std::string CLASS_NAME;
        virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      public: /* ------ SIGNALS ---------*/

        SignalPtr<dg::Vector, int> KpSIN; // is a 3d vector
        SignalPtr<dg::Vector, int> KdSIN; // is a 3d vector
        SignalPtr<dg::Vector, int> positionSIN;  // is a 3d vector
        SignalPtr<dg::Vector, int> desiredpositionSIN; // is a 3d vector
        SignalPtr<dg::Vector, int> biasedpositionSIN;
        SignalPtr<dg::Vector, int> velocitySIN; // is a 3d vector
        SignalPtr<dg::Vector, int> desiredvelocitySIN; // is a 3d vector
        SignalPtr<dg::Vector, int> biasedvelocitySIN;
        SignalPtr<dg::Vector, int> inertiaSIN;
        SignalPtr<dg::Vector, int> massSIN;
        SignalPtr<dg::Vector, int> angvelSIN;
        SignalPtr<dg::Vector, int> desiredangvelSIN;
        SignalPtr<dg::Vector, int> feedforwardforceSIN; // is a 3d vector
        SignalPtr<dg::Vector, int> feedforwardtorquesSIN;
        SignalPtr<dg::Vector, int> cntsensorSIN;
        SignalTimeDependent<dg::Vector, int> controlSOUT;
        SignalTimeDependent<dg::Vector, int> angcontrolSOUT;
        SignalTimeDependent<dg::Vector, int> SetPosBiasSOUT;
        SignalTimeDependent<dg::Vector, int> SetVelBiasSOUT;
        SignalTimeDependent<dg::Vector, int> ThrCntSensorSOUT;
      protected:

        double TimeStep;
        double& setsize(int dimension);
        dg::Vector& return_control_torques( dg::Vector& tau, int t);
        dg::Vector& return_angcontrol_torques( dg::Vector& angtau, int t);
        dg::Vector& set_pos_bias(dg::Vector& pos_bias, int t);
        dg::Vector& set_vel_bias(dg::Vector& vel_bias, int t);
        dg::Vector& threshold_cnt_sensor(dg::Vector& thr_cnt_sensor, int t);


        dg::Vector pos_error;
        dg::Vector vel_error;
        dg::Vector h_error;

        dg::Vector position_bias;
        dg::Vector velocity_bias;


        int init_flag_pos;
        int init_flag_vel;
        int isbiasset;
        int safetyswitch;
        int t_start;
        int bias_time;

      };
  } //namespace sot
}//namespace dynamic_graph

#endif // #ifndef
