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

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (com_impedance_control_EXPORTS)
#    define com_impedance_control_EXPORT __declspec(dllexport)
#  else
#    define com_impedance_control_EXPORT  __declspec(dllimport)
#  endif
#else
#  define com_impedance_control_EXPORT
#endif


namespace dynamicgraph{
  namespace sot {
/* ---------------------------------------------------------------------------*/
/* ---- CLASS ----------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/

    class com_impedance_control_EXPORT com_impedance_control
      : public Entity
      {
      public: /*------ Constructor --------*/

        com_impedance_control( const std::string &name);

      public: /*------ init --------*/

        void init( const double& step);

      public: /*------ CONSTANTS --------*/

        static const double TIME_STEP_DEFAULT;

      public: /* ----- ENTITY INHERITANCE -----*/

        static const std::string CLASS_NAME;
        virtual void display( std::ostream& os ) const;
        virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      public: /* ------ SIGNALS ---------*/

        SignalPtr<dg::Vector, int> KpSIN; // is a 7d vector
        SignalPtr<dg::Vector, int> KdSIN; // is a 6d vector
        SignalPtr<dg::Vector, int> positionSIN;  // is a 7d vector (xyzquat)
        SignalPtr<dg::Vector, int> desiredpositionSIN; // is a 7d vector (xyzquat)
        SignalPtr<dg::Vector, int> velocitySIN; // is a 6d vector
        SignalPtr<dg::Vector, int> desiredvelocitySIN; // is a 6d vector
        SignalTimeDependent<dg::Vector, int> controlSOUT;
        SignalTimeDependent<dg::Vector, int> pos_errorSOUT;
        SignalTimeDependent<dg::Vector, int> vel_errorSOUT;

      protected:

        double TimeStep;
        double& setsize(int dimension);
        dg::Vector& return_control_torques( dg::Vector& tau, int t);
        dg::Vector pos_error;
        dg::Vector vel_error;



      };
  } //namespace sot
}//namespace dynamic_graph

#endif // #ifndef
