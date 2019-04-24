// impedance controller implementation for COM (used for quadruped)
// Author : Avadesh Meduri
// Date : 25/03/19


#ifndef __SOT_com_impedance_Control_HH__
#define  __SOT_com_impedance_Control_HH__


/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/*QP */

#include <tsid/solvers/eiquadprog-fast.hpp>
// eigen-quadprog
#include <eigen-quadprog/QuadProg.h>
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
        SignalPtr<dg::Vector, int> KpAngSIN; // is a 3d vector
        SignalPtr<dg::Vector, int> KdAngSIN; // is a 3d vector
        SignalPtr<dg::Vector, int> KdSIN; // is a 3d vector
        SignalPtr<dg::Vector, int> positionSIN;  // is a 3d vector
        SignalPtr<dg::Vector, int> desiredpositionSIN; // is a 3d vector
        SignalPtr<dg::Vector, int> biasedpositionSIN;
        SignalPtr<dg::Vector, int> velocitySIN; // is a 3d vector
        SignalPtr<dg::Vector, int> desiredvelocitySIN; // is a 3d vector
        SignalPtr<dg::Vector, int> biasedvelocitySIN;
        SignalPtr<dg::Vector, int> inertiaSIN;
        SignalPtr<dg::Vector, int> massSIN;
        SignalPtr<dg::Vector, int> oriSIN; //base orientation 4d vector quaternion
        SignalPtr<dg::Vector, int> desoriSIN; //base orientation 4d vector quaternion
        SignalPtr<dg::Vector, int> angvelSIN;
        SignalPtr<dg::Vector, int> desiredangvelSIN;
        SignalPtr<dg::Vector, int> feedforwardforceSIN; // is a 3d vector
        SignalPtr<dg::Vector, int> feedforwardtorquesSIN;
        SignalPtr<dg::Vector, int> cntsensorSIN;
        SignalPtr<dg::Vector, int> thrcntvalueSIN; // thresholded contact sensor
        SignalPtr<dg::Vector, int> lqrerrorSIN;
        SignalPtr<dg::Vector, int> lqrgainSIN;
        SignalPtr<dg::Vector, int> lctrlSIN;
        SignalPtr<dg::Vector, int> actrlSIN;
        SignalPtr<dg::Matrix, int> hessSIN;
        SignalPtr<dg::Vector, int> g0SIN;
        SignalPtr<dg::Matrix, int> ceSIN;
        // SignalPtr<dg::Vector, int> ce0SIN;
        SignalPtr<dg::Matrix, int> ciSIN;
        SignalPtr<dg::Vector, int> ci0SIN;

        SignalTimeDependent<dg::Vector, int> controlSOUT;
        SignalTimeDependent<dg::Vector, int> angcontrolSOUT;
        SignalTimeDependent<dg::Vector, int> SetPosBiasSOUT;
        SignalTimeDependent<dg::Vector, int> SetVelBiasSOUT;
        SignalTimeDependent<dg::Vector, int> ThrCntSensorSOUT;
        SignalTimeDependent<dg::Vector, int> lqrcontrolSOUT;
        SignalTimeDependent<dg::Vector, int> wbcontrolSOUT; //whole body control



      protected:

        double TimeStep;
        double& setsize(int dimension);
        dg::Vector& return_control_torques( dg::Vector& tau, int t);
        dg::Vector& return_angcontrol_torques( dg::Vector& angtau, int t);
        dg::Vector& set_pos_bias(dg::Vector& pos_bias, int t);
        dg::Vector& set_vel_bias(dg::Vector& vel_bias, int t);
        dg::Vector& threshold_cnt_sensor(dg::Vector& thr_cnt_sensor, int t);
        dg::Vector& return_lqr_tau( dg::Vector& lqrtau, int t);
        dg::Vector& compute_end_eff_forces( dg::Vector & end_forces, int t);

        dg::Vector pos_error;
        dg::Vector vel_error;
        dg::Vector h_error;
        dg::Vector ori_error;

        dg::Vector position_bias;
        dg::Vector velocity_bias;

        dg::Vector delta_f;

        float w1;
        float w2;

        dg::Vector ce0;
        dg::Vector end_forces;
        // dg::Matrix ci;
        // dg::Vector ci0;


        tsid::solvers::EiquadprogFast m_solver;

        Eigen::QuadProgDense qp;

        Eigen::Quaternion<double> ori_quat;
        Eigen::Quaternion<double> des_ori_quat;
        Eigen::Quaternion<double> ori_error_quat;


        Eigen::Matrix<double, 3, 3> ori_se3;
        Eigen::Matrix<double, 3, 3> des_ori_se3;
        Eigen::Matrix<double, 3, 3> ori_error_se3; // refer to christian ott paper for definitions (Rdb)

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
