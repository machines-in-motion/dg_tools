// receding reactive LQR implementation for solo. The desired trajectory is obtained
// from the centroidal momentum controller.

//Author : Avadesh Meduri
// Date: 29/04/2019

#include <iostream>
#include "dg_tools/ComImpedanceControl/reactive_lqr_controller.hpp"

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
using namespace std;
using namespace Eigen;


/*------------------------- Python name during import --------------------*/
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ReactiveLQRController, "ReactiveLQRController");


ReactiveLQRController::ReactiveLQRController(const std::string & name)
  : Entity(name)
  ,TimeStep(0)
  ,com_posSIN(NULL, "ReactiveLQRController("+name+")::input(vector)::com_pos")
  ,com_velSIN(NULL, "ReactiveLQRController("+name+")::input(vector)::com_vel")
  ,com_oriSIN(NULL, "ReactiveLQRController("+name+")::input(vector)::com_ori")
  ,com_ang_velSIN(NULL, "ReactiveLQRController("+name+")::input(vector)::com_ang_vel")
  ,end_eff_pos_12dSIN(NULL, "ReactiveLQRController("+name+")::input(vector)::end_eff_pos")
  ,des_fffSIN(NULL, "ReactiveLQRController("+name+")::input(vector)::des_fff")
  ,cnt_valueSIN(NULL, "ReactiveLQRController("+name+")::input(vector)::cnt_value")
  ,cnt_sensor_valueSIN(NULL, "ReactiveLQRController("+name+")::input(vector)::cnt_sensor_value")
  ,state_vectorSIN(NULL, "ReactiveLQRController("+name+")::input(vector)::state_vector")
  ,massSIN(NULL, "ReactiveLQRController("+name+")::input(vector)::mass")
  ,inertiaSIN(NULL, "ReactiveLQRController("+name+")::input(Matrix)::inertia")
  ,horizonSIN(NULL, "ReactiveLQRController("+name+")::input(Vector)::horizon")
  ,qSIN(NULL, "ReactiveLQRController("+name+")::input(Matrix)::q")
  ,rSIN(NULL, "ReactiveLQRController("+name+")::input(Matrix)::r")


  ,lqr_gainsSOUT(boost::bind(&ReactiveLQRController::return_lqr_gains_, this, _1, _2),
        com_posSIN << com_velSIN << com_oriSIN << com_ang_velSIN << end_eff_pos_12dSIN <<
        des_fffSIN << cnt_valueSIN << cnt_sensor_valueSIN << massSIN <<
        inertiaSIN << horizonSIN << qSIN << rSIN,
                    "ReactiveLQRController("+name+")::output(Matrix)::return_lqr_gains")

  {
    init(TimeStep);
    // initializing matrix sizes

    Entity::signalRegistration(
      com_posSIN << com_velSIN << com_oriSIN << com_ang_velSIN << end_eff_pos_12dSIN <<
      des_fffSIN << cnt_valueSIN << cnt_sensor_valueSIN << state_vectorSIN << massSIN <<
      inertiaSIN << horizonSIN << qSIN << rSIN << lqr_gainsSOUT

    );
  }

MatrixXd ReactiveLQRController::
  compute_r_cross_mat_(VectorXd end_eff_pos, Vector3d com_pos ){
    // computes the R matrix for cross product between R and F
    // R = [[0, -rz, ry],
    //      [rz, 0, -rx],
    //      -ry, rx, 0]]

    assert(end_eff_pos.size()==3);

    R_cross_mat << 0.0, -1.0*(end_eff_pos(2)-com_pos(2)), end_eff_pos(1)-com_pos(1),
                   end_eff_pos(2)-com_pos(2), 0.0, -1.0*(end_eff_pos(0)-com_pos(0)),
                   -1.0*(end_eff_pos(1)-com_pos(1)), (end_eff_pos(0)-com_pos(0)), 0.0;

    return R_cross_mat;
  }



VectorXd ReactiveLQRController::
  compute_dyn_(Vector3d com_pos, Vector3d com_vel,  Quaternion<double> ori, Vector3d com_ang_vel, VectorXd end_eff_pos_12d,
              VectorXd des_fff, Vector4d cnt_value, double mass, MatrixXd inertia ,MatrixXd& A_t, MatrixXd& B_t){
    //computes dynamic for the urrent time step
    // xd = A_t * X + B_t*U

    // should be bound by an init flag
    omega.resize(4,4);
    mass_inv_identity.resize(3,3);
    A_t.resize(13,13);
    B_t.resize(13,12);
    R_cross_mat.resize(3,3);
    x.resize(13);

    // cout << "asserting in dyn computation" << endl;

    assert(com_pos.size() == 3);
    assert(com_ang_vel.size() == 3);
    assert(end_eff_pos_12d.size() == 12);
    assert(cnt_value.size() == 4);
    // assert(mass.size() == 1);
    // cout << "asserting done in dyn computation" << endl;

    // Make sure omega is 4*4 matrix
    omega << 0, com_ang_vel(2), -1*com_ang_vel(1), com_ang_vel(0),
            -1*com_ang_vel(2), 0, com_ang_vel(0), com_ang_vel(1),
            com_ang_vel(1), -1*com_ang_vel(0), 0, com_ang_vel(2),
            -1*com_ang_vel(0), -1*com_ang_vel(1), -1*com_ang_vel(2), 0;

    // cout << "completed defining omega ..." << endl;
    A_t << Matrix::Zero(3,3), Matrix::Identity(3,3), Matrix::Zero(3,4), Matrix::Zero(3,3),
           Matrix::Zero(3,3), Matrix::Zero(3,3), Matrix::Zero(3,4), Matrix::Zero(3,3),
           Matrix::Zero(4,3), Matrix::Zero(4,3),    0.5*omega,      Matrix::Zero(4,3),
           Matrix::Zero(3,3), Matrix::Zero(3,3), Matrix::Zero(3,4), Matrix::Zero(3,3);

    // cout << "completed defining A" << endl;

    mass_inv_identity << 1/mass , 0.0, 0.0,
                          0.0,  1/mass, 0.0,
                          0.0, 0.0,  1/mass ;

    // cout << "completed defining mass matrix" << endl;

    inertia_global_inv = (ori.toRotationMatrix() * inertia * ori.toRotationMatrix().transpose()).inverse();


    R_cross_mat_fl = inertia_global_inv * this->compute_r_cross_mat_(end_eff_pos_12d.segment(0,3), com_pos);
    R_cross_mat_fr = inertia_global_inv * this->compute_r_cross_mat_(end_eff_pos_12d.segment(3,3), com_pos);
    R_cross_mat_hl = inertia_global_inv * this->compute_r_cross_mat_(end_eff_pos_12d.segment(6,3), com_pos);
    R_cross_mat_hr = inertia_global_inv * this->compute_r_cross_mat_(end_eff_pos_12d.segment(9,3), com_pos);

    // cout << "completed calcultatin cross matrixes" << endl;

    B_t << Matrix::Zero(3,3), Matrix::Zero(3,3), Matrix::Zero(3,3), Matrix::Zero(3,3),
           cnt_value(0) * mass_inv_identity, cnt_value(1) * mass_inv_identity, cnt_value(2) * mass_inv_identity, cnt_value(3) * mass_inv_identity,
           Matrix::Zero(4,3), Matrix::Zero(4,3), Matrix::Zero(4,3), Matrix::Zero(4,3),
           cnt_value(0) * R_cross_mat_fl , cnt_value(1) * R_cross_mat_fr, cnt_value(2) * R_cross_mat_hl, cnt_value(3) * R_cross_mat_hr;

   // cout << B_t << "\n\n\n" <<endl;


   x << com_pos(0), com_pos(1), com_pos(2), com_vel(0), com_vel(1), com_vel(2),
        ori.vec()[0], ori.vec()[1], ori.vec()[2], ori.w(), com_ang_vel(0), com_ang_vel(1), com_ang_vel(2);


    return A_t * x + B_t * des_fff;



  }


void ReactiveLQRController::
  compute_lin_dyn_(Eigen::MatrixXd lin_A_t, Eigen::MatrixXd lin_B_t){
  // computes the linearized A and B in the taylor expansion
  // Xd = lin_A_t*X + lin_B_t*U

  }

void ReactiveLQRController::
  compute_num_lin_dyn_(
          Vector3d com_pos_t, VectorXd com_vel_t, Vector3d com_ang_vel_t,
          Quaternion<double> ori_t, VectorXd end_eff_pos_12d_t, Vector4d cnt_value_t,
          Vector3d com_pos_t1, Vector3d com_vel_t1, Vector3d com_ang_vel_t1, Quaternion<double> ori_t1,
          VectorXd end_eff_pos_12d_t1, Vector4d cnt_value_t1,
          VectorXd des_fff, double mass, MatrixXd inertia,
          MatrixXd& lin_A_t, MatrixXd& lin_B_t){
  // computes the linearized A and B using numerical differentiation


    // should be put inside an init flag
      unit_vec.resize(13);
      lin_A_t.resize(13,13);
      lin_B_t.resize(13,12);

      xd_t = this->compute_dyn_(com_pos_t, com_vel_t, ori_t, com_ang_vel_t, end_eff_pos_12d_t, des_fff, cnt_value_t,
                        mass, inertia, A_t, B_t);
     //
      // cout << "completed defining state vector" << endl;
      // lin_A = (dA/dx)*X^T + A + (dB/dx)*U^T

     // diffrentiating with repect to com_pos_x
     for(int i = 0; i < 3; i ++){
       com_pos_pd = com_pos_t;
       com_pos_pd(i) = com_pos_t1(i);
       lin_A_t.col(i) = (1/(com_pos_t1(i) - com_pos_t(i)))*(this->compute_dyn_(com_pos_pd,com_vel_t, ori_t, com_ang_vel_t, end_eff_pos_12d_t, des_fff, cnt_value_t,
                         mass, inertia, A_t, B_t) - xd_t);

        com_vel_pd = com_vel_t;
        com_vel_pd(i) = com_vel_t1(i);
        lin_A_t.col(i+3) = (1/(com_vel_t1(i) - com_vel_t(i)))*(this->compute_dyn_(com_pos_t,com_vel_pd, ori_t, com_ang_vel_t, end_eff_pos_12d_t, des_fff, cnt_value_t,
                        mass, inertia, A_t, B_t) - xd_t);

        com_ori_pd = ori_t;
        com_ori_pd.vec()[i] = ori_t1.vec()[i];
        lin_A_t.col(i + 6) = (1/(ori_t1.vec()[i] - ori_t.vec()[i]))*(this->compute_dyn_(com_pos_t,com_vel_t, com_ori_pd, com_ang_vel_t, end_eff_pos_12d_t, des_fff, cnt_value_t,
                          mass, inertia, A_t, B_t) - xd_t);

        com_ang_vel_pd = com_ang_vel_t;
        com_ang_vel_pd(i) = com_ang_vel_t1(i);
        lin_A_t.col(i+10) = (1/(com_ang_vel_t1(i) - com_ang_vel_t(i)))*(this->compute_dyn_(com_pos_t,com_vel_t, ori_t, com_ang_vel_pd, end_eff_pos_12d_t, des_fff, cnt_value_t,
                        mass, inertia, A_t, B_t) - xd_t);

      }

      com_ori_pd = ori_t;
      com_ori_pd.w() = ori_t1.w();
      lin_A_t.col(9) = (1/(ori_t1.w() - ori_t.w()))*(this->compute_dyn_(com_pos_t,com_vel_t, com_ori_pd, com_ang_vel_t, end_eff_pos_12d_t, des_fff, cnt_value_t,
                        mass, inertia, A_t, B_t) - xd_t);


      // cout << "completed computing lin_A" << endl;
      //
      lin_B_t = B_t;

      // cout << lin_B_t << "\n\n\n" << endl;
      // cout << lin_B_t << endl;


  }

void ReactiveLQRController::
  compute_lqr_gains_(MatrixXd Q, MatrixXd R, MatrixXd lin_A, MatrixXd lin_B,
                    MatrixXd P_prev, MatrixXd &P, MatrixXd &K){

      K.resize(12,13);
      // K = -(R + B ^T *P_prev * B)^(-1)* B^T * P_prev * A
      K = -1*(((R + lin_B.transpose() * P_prev * lin_B).inverse()) * lin_B.transpose() * P_prev * lin_A);
      // making each element negative of its value
      // K = Matrix::Zero(12,13) - K;
      // cout << K << endl;
      P = Q + K.transpose()*R*K + (lin_A + lin_B*K).transpose() * P_prev * (lin_A + lin_B*K);

      // cout << ((R + lin_B.transpose() * P_prev * lin_B).inverse()) * lin_B.transpose() * P_prev * lin_A  << "\n\n\n\n"<< endl;


  }


dynamicgraph::Matrix& ReactiveLQRController::
  return_lqr_gains_(dynamicgraph::Matrix & lqr_gains, int t){

    sotDEBUGIN(15);

    const dynamicgraph::Vector& com_pos = com_posSIN(t);
    const dynamicgraph::Vector& com_vel = com_velSIN(t);
    const dynamicgraph::Vector& com_ang_vel = com_ang_velSIN(t);
    const dynamicgraph::Vector& com_ori = com_oriSIN(t);
    const dynamicgraph::Vector& end_eff_pos_12d = end_eff_pos_12dSIN(t);
    const dynamicgraph::Vector& des_fff_12d = des_fffSIN(t);
    const dynamicgraph::Vector& cnt_value = cnt_valueSIN(t);
    // const dynamicgraph::Vector& cnt_sensor_value = cnt_sensor_valueSIN(t);
    // const dynamicgraph::Vector& state_vector = state_vectorSIN(t);
    const dynamicgraph::Vector& mass = massSIN(t);
    const dynamicgraph::Matrix& inertia = inertiaSIN(t);

    const dynamicgraph::Vector& horizon = horizonSIN(t);
    const dynamicgraph::Matrix& Q = qSIN(t);
    const dynamicgraph::Matrix& R = rSIN(t);

    P_prev = Matrix::Zero(13,13);


    // cout << "asserting in return_lqr_gains" << endl;

    assert(com_pos.size() == 3*(horizon[0] + 1));
    assert(com_vel.size() == 3*(horizon[0] + 1));
    assert(com_ang_vel.size() == 3*(horizon[0] + 1));
    assert(com_ori.size() == 4*(horizon[0] + 1));
    assert(end_eff_pos_12d.size() == 12*(horizon[0] + 1));
    assert(des_fff_12d.size() == 12*(horizon[0] + 1));
    assert(cnt_value.size() == 4*(horizon[0] + 1));

    // cout << "completed asserting in return_lqr_gains" << endl;


    // cout << horizon[0] << endl;

    // backward pass of lqr
    for (int h=1; h < horizon[0]+1; h++){

      com_pos_t = com_pos.segment(3*(horizon[0] - h), 3);
      com_vel_t = com_vel.segment(3*(horizon[0] - h), 3);
      com_ang_vel_t = com_ang_vel.segment(3*(horizon[0] - h), 3);
      ori_tmp = com_ori.segment(4*(horizon[0] - h), 4);
      com_ori_t.w() = ori_tmp[3];
      com_ori_t.vec() = ori_tmp.segment(0,3);
      end_eff_pos_12d_t = end_eff_pos_12d.segment(12*(horizon[0] - h), 12);
      cnt_value_t = cnt_value.segment(4*(horizon[0] - h), 4);

      com_pos_t1 = com_pos.segment(3*(horizon[0]-h+1), 3);
      com_vel_t1 = com_vel.segment(3*(horizon[0]-h+1), 3);
      com_ang_vel_t1 = com_ang_vel.segment(3*(horizon[0]-h+1), 3);
      ori_tmp = com_ori.segment(4*(horizon[0]-h+1), 4);
      com_ori_t1.w() = ori_tmp[3];
      com_ori_t1.vec() = ori_tmp.segment(0,3);
      end_eff_pos_12d_t1 = end_eff_pos_12d.segment(12*(horizon[0]-h+1), 12);
      cnt_value_t1 = cnt_value.segment(4*(horizon[0]-h+1), 4);
      //
      //
      des_fff_t = des_fff_12d.segment(12*(horizon[0]- h), 12);

      // xd_t = this->compute_dyn_(com_pos_t, com_ang_vel_t, com_ori_t, com_ang_vel_t, end_eff_pos_12d_t, des_fff_t, cnt_value_t,
      //                     mass[0], inertia, A_t, B_t);

      this->compute_num_lin_dyn_(com_pos_t, com_vel_t, com_ang_vel_t, com_ori_t, end_eff_pos_12d_t, cnt_value_t,
                                 com_pos_t1, com_vel_t1, com_ang_vel_t1, com_ori_t1, end_eff_pos_12d_t1, cnt_value_t1,
                                 des_fff_t, mass[0], inertia, lin_A_ht, lin_B_ht);
      //
      this->compute_lqr_gains_(Q, R, lin_A_ht, lin_B_ht, P_prev, P, K );
      // //
      //
      P_prev = P;
      // cout << P_prev << "\n\n\n" <<endl;


    }

    lqr_gains = K;

    cout << K << "\n\n\n" << endl;

    // lqr_gains = Matrix::Zero(12,13);

    sotDEBUGOUT(15);
    return lqr_gains;

  }
