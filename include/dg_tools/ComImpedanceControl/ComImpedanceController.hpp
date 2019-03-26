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
        SignalPtr<dg::Vector, int> positionSIN;  // is a 3d vector (xyzquat)
        SignalPtr<dg::Vector, int> desiredpositionSIN; // is a 3d vector (xyzquat)
        SignalPtr<dg::Vector, int> velocitySIN; // is a 3d vector
        SignalPtr<dg::Vector, int> desiredvelocitySIN; // is a 3d vector
        SignalPtr<dg::Vector,int> feedforwardforceSIN; // is a 3d vector
        SignalTimeDependent<dg::Vector, int> controlSOUT;
        SignalTimeDependent<dg::Vector, int> set_biasSOUT;

      protected:

        double TimeStep;
        double& setsize(int dimension);
        dg::Vector& return_control_torques( dg::Vector& tau, int t);
        bool set_bias(dg::Vector& pos_bias, dg::Vector& vel_bias, int t);
        dg::Vector pos_error;
        dg::Vector vel_error;
        dg::Vector vicon_pos_bias; // can be renamed to other sensors
        dg::Vector vicon_vel_bias; // can be renamed to other sensors


      };
  } //namespace sot
}//namespace dynamic_graph

#endif // #ifndef
