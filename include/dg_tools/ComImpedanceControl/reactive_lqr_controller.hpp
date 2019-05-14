// receding reactive LQR implementation for solo. The desired trajectory is obtained
// from the centroidal momentum controller.

//Author : Avadesh Meduri
// Date: 29/04/2019


#ifndef __SOT_reactive_lqr_controller_HH__
#define __SOT_reactive_lqr_controller_HH__

/* Math */

#include <math.h>

/* Matrix */
#include <dynamic-graph/linear-algebra.h>
namespace dg = dynamicgraph;

/* SOT */
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/entity.h>

namespace dynamicgraph{
  namespace sot {
    /* -----------------------------------------------------------------------*/
    /* ---------- CLASS ------------------------------------------------------*/
    /* -----------------------------------------------------------------------*/

    class ReactiveLQRController
      : public Entity
      {
      public: /*-------------- Constructor ---------------------------------*/

        ReactiveLQRController( const std::string &name);

        void init (const double& step){};

      public: /*----------------- Constant ----------------------------------*/

        static const double TIME_STEP_DEFAULT;

      public: /* ----- ENTITY INHERITANCE -----*/

        static const std::string CLASS_NAME;
        virtual const std::string& getClassName( void ) const {return CLASS_NAME;}

      public: /* ---- SIGNALS ----------------------------------------------*/

        SignalPtr<dg::Vector, int> com_posSIN;  // is a 3d vector
        SignalPtr<dg::Vector, int> com_velSIN; // is a 3d vector
        SignalPtr<dg::Vector, int> com_oriSIN; //base orientation 4d vector quaternion
        SignalPtr<dg::Vector, int> com_ang_velSIN;
        SignalPtr<dg::Vector, int> end_eff_pos_12dSIN;
        SignalPtr<dg::Vector, int> des_fffSIN;
        SignalPtr<dg::Vector, int> cnt_valueSIN;
        SignalPtr<dg::Vector, int> cnt_sensor_valueSIN;
        SignalPtr<dg::Vector, int> state_vectorSIN;
        SignalPtr<dg::Vector, int> massSIN;
        SignalPtr<dg::Matrix, int> inertiaSIN;

        SignalPtr<dg::Vector, int> horizonSIN;
        SignalPtr<dg::Matrix, int> qSIN;
        SignalPtr<dg::Matrix, int> rSIN;


        SignalTimeDependent<dg::Matrix, int> lqr_gainsSOUT;


      protected:

        double TimeStep;
        double& setsize(int dimension);

        Eigen::VectorXd compute_dyn_(Eigen::Vector3d com_pos, Eigen::Vector3d com_vel, Eigen::Quaternion<double> ori, Eigen::Vector3d com_ang_vel,
                         Eigen::VectorXd end_eff_pos_12d, Eigen::VectorXd des_fff, Eigen::Vector4d cnt_value, double mass,
                        Eigen::MatrixXd inertia, Eigen::MatrixXd& A_t, Eigen::MatrixXd& B_t);
        void compute_lin_dyn_(Eigen::MatrixXd lin_A_t, Eigen::MatrixXd lin_B_t);
        void compute_num_lin_dyn_(
                  Eigen::Vector3d com_pos_t, Eigen::VectorXd com_vel, Eigen::Vector3d com_ang_vel_t,
                  Eigen::Quaternion<double> ori_t, Eigen::VectorXd end_eff_pos_12d_t,
                  Eigen::Vector4d cnt_value_t,
                  Eigen::Vector3d com_pos_t1, Eigen::Vector3d com_vel_t1, Eigen::Vector3d com_ang_vel_t1,
                  Eigen::Quaternion<double> ori_t1,
                  Eigen::VectorXd end_eff_pos_12d_t1, Eigen::Vector4d cnt_value_t1,
                  Eigen::VectorXd des_fff, double mass, Eigen::MatrixXd inertia,
                  Eigen::MatrixXd& lin_A_t, Eigen::MatrixXd& lin_B_t);


        Eigen::MatrixXd compute_r_cross_mat_(Eigen::VectorXd end_eff_pos, Eigen::Vector3d com_pos );

        void compute_lqr_gains_(Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd lin_A,
            Eigen::MatrixXd lin_B, Eigen::MatrixXd P_prev, Eigen::MatrixXd &P, Eigen::MatrixXd &K);

        Eigen::MatrixXd omega;
        // Eigen::MatrixXd com_acc;
        Eigen::MatrixXd inertia_global_inv; // inertia of solo body in global co ordinate (RIR^T)
        Eigen::MatrixXd mass_inv_identity; // 1/m * I
        Eigen::MatrixXd R_cross_mat;
        Eigen::MatrixXd R_cross_mat_fl;
        Eigen::MatrixXd R_cross_mat_fr;
        Eigen::MatrixXd R_cross_mat_hl;
        Eigen::MatrixXd R_cross_mat_hr;

        // to compute the liear dynamics;
        Eigen::MatrixXd A_t;
        Eigen::MatrixXd B_t;
        Eigen::MatrixXd A_t1; // t+1 step
        Eigen::MatrixXd B_t1; // t+1 step
        Eigen::VectorXd x;
        Eigen::VectorXd unit_vec;
        Eigen::Vector3d com_pos_pd; // Partial derivative
        Eigen::Vector3d com_vel_pd;
        Eigen::Vector3d com_ang_vel_pd;
        Eigen::Quaternion<double> com_ori_pd;


        // dynamic grpah signal
        dynamicgraph::Matrix& return_lqr_gains_(dynamicgraph::Matrix & lqr_gains, int t);
        Eigen::MatrixXd P_prev;
        Eigen::MatrixXd K_prev;

        Eigen::MatrixXd P;
        Eigen::MatrixXd K;

        // Backward pass of LQR
        //current timestep

        Eigen::VectorXd com_pos_t;
        Eigen::VectorXd com_vel_t;
        Eigen::VectorXd com_ang_vel_t;
        Eigen::VectorXd ori_tmp;
        Eigen::Quaternion<double> com_ori_t;
        Eigen::VectorXd des_fff_t;
        Eigen::VectorXd cnt_value_t;
        Eigen::VectorXd end_eff_pos_12d_t;
        Eigen::VectorXd xd_t;


        // next timestep
        Eigen::VectorXd com_pos_t1;
        Eigen::VectorXd com_vel_t1;
        Eigen::VectorXd com_ang_vel_t1;
        Eigen::Quaternion<double> com_ori_t1;
        Eigen::VectorXd des_fff_t1;
        Eigen::VectorXd cnt_value_t1;
        Eigen::VectorXd end_eff_pos_12d_t1;
        Eigen::VectorXd xd_t1;

        Eigen::MatrixXd lin_A_ht;
        Eigen::MatrixXd lin_B_ht;




      };
  }
}

#endif
