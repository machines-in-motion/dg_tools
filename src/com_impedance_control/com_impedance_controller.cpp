// impedance controller implementation for COM (used for quadruped)
// Author : Avadesh Meduri
// Date : 25/03/19

#include <iostream>
#include "dg_tools/ComImpedanceControl/ComImpedanceController.hpp"

/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

using namespace dynamicgraph::sot;
using namespace dynamicgraph;
using namespace std;


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ComImpedanceControl, "ComImpedanceControl");


ComImpedanceControl::ComImpedanceControl(const std::string & name)
  :Entity(name)
  ,TimeStep(0)
  ,KpSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::Kp")
  ,KpAngSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::Kp_ang")
  ,KdAngSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::Kd_ang")
  ,KdSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::Kd")
  ,positionSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::position")
  ,desiredpositionSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_pos")
  ,biasedpositionSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::biased_pos")
  ,velocitySIN(NULL, "ComImpedanceControl("+name+")::input(vector)::velocity")
  ,desiredvelocitySIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_vel")
  ,biasedvelocitySIN(NULL, "ComImpedanceControl("+name+")::input(vector)::biased_vel")
  ,feedforwardforceSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_fff")
  ,inertiaSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::inertia")
  ,massSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::mass")
  ,oriSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::ori")
  ,desoriSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_ori")
  ,angvelSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::angvel")
  ,desiredangvelSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_ang_vel")
  ,feedforwardtorquesSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::des_fft")
  ,cntsensorSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::cnt_sensor")
  ,thrcntvalueSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::thr_cnt_value")
  ,lqrerrorSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::lqr_error")
  ,lqrgainSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::lqr_gain")
  ,lctrlSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::lctrl")
  ,actrlSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::actrl")
  ,hessSIN(NULL, "ComImpedanceControl("+name+")::input(Matrix)::hess")
  ,g0SIN(NULL, "ComImpedanceControl("+name+")::input(vector)::g0")
  ,ceSIN(NULL, "ComImpedanceControl("+name+")::input(Matrix)::ce")
  // ,ce0SIN(NULL, "ComImpedanceControl("+name+")::input(vector)::ce0")
  ,ciSIN(NULL, "ComImpedanceControl("+name+")::input(Matrix)::ci")
  ,ci0SIN(NULL, "ComImpedanceControl("+name+")::input(vector)::ci0")
  ,regSIN(NULL, "ComImpedanceControl("+name+")::input(Matrix)::reg")
  ,leglengthflSIN(NULL, "ComImpedanceControl("+name+")::input(Matrix)::leg_length_fl")
  ,leglengthhlSIN(NULL, "ComImpedanceControl("+name+")::input(Matrix)::leg_length_hl")
  ,absendeffposSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::abs_end_eff_pos")
  ,absendeffvelSIN(NULL, "ComImpedanceControl("+name+")::input(vector)::abs_end_eff_vel")

  ,controlSOUT( boost::bind(&ComImpedanceControl::return_control_torques, this, _1,_2),
                KpSIN << KdSIN << positionSIN << desiredpositionSIN <<
                velocitySIN << desiredvelocitySIN << feedforwardforceSIN ,
                "ComImpedanceControl("+name+")::output(vector)::tau")
  ,angcontrolSOUT( boost::bind(&ComImpedanceControl::return_angcontrol_torques, this, _1,_2),
                  KpSIN << KdSIN << inertiaSIN <<angvelSIN << desiredangvelSIN
                  << feedforwardtorquesSIN,
                    "ComImpedanceControl("+name+")::output(vector)::angtau")
  ,SetPosBiasSOUT( boost::bind(&ComImpedanceControl::set_pos_bias, this, _1,_2),
                positionSIN , "ComImpedanceControl("+name+")::output(vector)::set_pos_bias")
  ,SetVelBiasSOUT( boost::bind(&ComImpedanceControl::set_vel_bias, this, _1,_2),
                velocitySIN, "ComImpedanceControl("+name+")::output(vector)::set_vel_bias")
  ,ThrCntSensorSOUT( boost::bind(&ComImpedanceControl::threshold_cnt_sensor, this, _1, _2),
                cntsensorSIN, "ComImpedanceControl("+name+")::output(vector)::thr_cnt_sensor")
  ,lqrcontrolSOUT( boost::bind(&ComImpedanceControl::return_lqr_tau, this, _1,_2),
                    lqrgainSIN,
                   "ComImpedanceControl("+name+")::output(vector)::lqrtau")
  ,endefflqrcontrolSOUT( boost::bind(&ComImpedanceControl::return_end_eff_lqr_tau, this, _1,_2),
                     lqrgainSIN,
                    "ComImpedanceControl("+name+")::output(vector)::end_eff_lqr_tau")
  ,wbcontrolSOUT( boost::bind(&ComImpedanceControl::compute_end_eff_forces, this, _1, _2),
                      lctrlSIN << actrlSIN << hessSIN << g0SIN << ceSIN
                      << ciSIN << ci0SIN << absendeffvelSIN,
                    "ComImpedanceControl("+name+")::output(vector)::wbctrl")
  ,descomposSOUT( boost::bind(&ComImpedanceControl::compute_des_com_pos, this, _1, _2),
                      leglengthflSIN << leglengthhlSIN,
                    "ComImpedanceControl("+name+")::output(vector)::compute_des_com_pos")


  ,isbiasset(0)
  ,safetyswitch(0)
  ,init_flag_pos(1)
  ,init_flag_vel(1)
  ,bias_time(100)
{
  init(TimeStep);
  Entity::signalRegistration(
    positionSIN << velocitySIN << SetPosBiasSOUT << SetVelBiasSOUT << KpSIN << KdSIN <<
    KdAngSIN << KpAngSIN << inertiaSIN << biasedpositionSIN << biasedvelocitySIN << desiredpositionSIN << massSIN << cntsensorSIN
    << ThrCntSensorSOUT  << desiredvelocitySIN << feedforwardforceSIN <<  controlSOUT <<
    oriSIN << desoriSIN << angvelSIN << desiredangvelSIN << endefflqrcontrolSOUT
    << feedforwardtorquesSIN << thrcntvalueSIN << absendeffposSIN << absendeffvelSIN
    << angcontrolSOUT << lqrerrorSIN << lqrgainSIN << lqrcontrolSOUT << lctrlSIN
    << actrlSIN << hessSIN << g0SIN << ceSIN << ciSIN << ci0SIN << regSIN <<
    wbcontrolSOUT << leglengthflSIN << leglengthhlSIN << descomposSOUT
  );
}

dynamicgraph::Vector& ComImpedanceControl::
  return_lqr_tau( dynamicgraph::Vector &lqrtau, int t){
    sotDEBUGIN(15);

    /** This method carries out a matrix multiplication of lqr gains
    obtained from the dynamic planner.
    lqr_matrix * [[com_error], [lmom_error], [ori_error], [amom_error]] **/

    const dynamicgraph::Vector& lqr_vector = lqrgainSIN(t);
    const dynamicgraph::Vector& position = biasedpositionSIN(t);
    const dynamicgraph::Vector& des_pos = desiredpositionSIN(t);
    const dynamicgraph::Vector& velocity = biasedvelocitySIN(t);
    const dynamicgraph::Vector& des_vel = desiredvelocitySIN(t);
    const dynamicgraph::Vector& des_fff = feedforwardforceSIN(t);
    const dynamicgraph::Vector& des_ori = desoriSIN(t);
    const dynamicgraph::Vector& ori = oriSIN(t);
    const dynamicgraph::Vector& omega = angvelSIN(t);
    const dynamicgraph::Vector& des_omega = desiredangvelSIN(t);
    const dynamicgraph::Vector& hd_des = feedforwardtorquesSIN(t);

    lqrtau.resize(6);lqrtau.setZero();

    assert(lqr_vector.size()==78);

    /*------- computing delta X -----------------------------*/
    delta_x.resize(13);
    lqr_pos_error.array() = position.array() - des_pos.array();
    lqr_vel_error.array() = velocity.array() - des_vel.array();
    lqr_ang_vel_error.array() = omega.array() - des_omega.array();

    des_ori_quat.w() = des_ori[3];
    des_ori_quat.vec()[0] = des_ori[0];
    des_ori_quat.vec()[1] = des_ori[1];
    des_ori_quat.vec()[2] = des_ori[2];

    ori_quat.w() = ori[3];
    ori_quat.vec()[0] = ori[0];
    ori_quat.vec()[1] = ori[1];
    ori_quat.vec()[2] = ori[2];
    //
    des_ori_se3 = des_ori_quat.toRotationMatrix();
    ori_se3 = ori_quat.toRotationMatrix();
    //
    ori_error_se3 = des_ori_se3.transpose() * ori_se3;
    ori_error_quat = ori_error_se3;

    lqr_ori_error(0) = ori_error_quat.vec()[0];
    lqr_ori_error(1) = ori_error_quat.vec()[1];
    lqr_ori_error(2) = ori_error_quat.vec()[2];
    lqr_ori_error(3) = ori_error_quat.w();

    delta_x << lqr_pos_error, lqr_vel_error, lqr_ori_error, lqr_ang_vel_error;

    /*--------- reshaping the lqr_vector into a 6*13 matrix ------*/

    K.resize(6,13);

    // cout << lqr_vector << endl;

    K.row(0) = lqr_vector.segment(0,13),
    K.row(1) = lqr_vector.segment(13,13),
    K.row(2) = lqr_vector.segment(26,13),
    K.row(3) = lqr_vector.segment(39,13),
    K.row(4) = lqr_vector.segment(52,13),
    K.row(5) = lqr_vector.segment(65,13);



    // K << -200, 0, 0, -20, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //       0, 0, -200, 0, 0, -20, 0, 0, 0, 0, 0, 0, 0,
    //       0, 0, 0, 0, 0, 0, -130, 0, 0, 0, -.5, 0, 0,
    //       0, 0, 0, 0, 0, 0, 0,-130,0 ,0, 0, -1., 0,
    //       0, 0, 0, 0, 0, 0, 0, 0, -130,0, 0, 0, -.5;

    // cout << K << "\n" <<endl;

    /**** computing the centroidal forces at current timestep ----*/
    // F = F* - K*delta_x
    f_des.resize(6);
    f_des << des_fff, hd_des;

    lqrtau = f_des + K * delta_x;

    // lqrtau(3) = lqr_ori_error(0) * lqrtau(3);
    // lqrtau(4) = lqr_ori_error(1) * lqrtau(4);
    // lqrtau(5) = lqr_ori_error(2) * lqrtau(5);

    sotDEBUGOUT(15);
    return lqrtau;
    }

    dynamicgraph::Vector& ComImpedanceControl::
      return_angcontrol_torques( dynamicgraph::Vector &angtau, int t){
        sotDEBUGIN(15);

        const dynamicgraph::Vector& Kp_ang = KpAngSIN(t);
        const dynamicgraph::Vector& Kd_ang = KdAngSIN(t);
        const dynamicgraph::Vector& inertia = inertiaSIN(t);
        const dynamicgraph::Vector& des_ori = desoriSIN(t);
        const dynamicgraph::Vector& ori = oriSIN(t);
        const dynamicgraph::Vector& omega = angvelSIN(t);
        const dynamicgraph::Vector& des_omega = desiredangvelSIN(t);
        const dynamicgraph::Vector& hd_des = feedforwardtorquesSIN(t);
        std::cout << "LhumLqr ori " << ori.array();
        std::cout << "LhumLqr desori " << des_ori.array();

        if (isbiasset){
          /*----- assertions of sizes -------------*/

          assert(Kp_ang.size() == 3);
          assert(Kd_ang.size() == 3);
          assert(des_ori.size() == 4);
          assert(ori.size() == 4);
          assert(omega.size() == 3);
          assert(des_omega.size() == 3);
          assert(hd_des.size()==3);
          assert(inertia.size()==3);

          des_ori_quat.w() = des_ori[3];
          des_ori_quat.vec()[0] = des_ori[0];
          des_ori_quat.vec()[1] = des_ori[1];
          des_ori_quat.vec()[2] = des_ori[2];

          ori_quat.w() = ori[3];
          ori_quat.vec()[0] = ori[0];
          ori_quat.vec()[1] = ori[1];
          ori_quat.vec()[2] = ori[2];

          des_ori_se3 = des_ori_quat.toRotationMatrix();
          ori_se3 = ori_quat.toRotationMatrix();

          ori_error_se3 = des_ori_se3.transpose() * ori_se3;
          std::cout << "LhumOri e " << ori_error_se3 << std::endl;
          ori_error_quat = ori_error_se3;
          std::cout << "LhumOri q " << ori_error_quat.vec() << "  " << ori_error_quat.w() << std::endl;

          //todo: multiply as matrix

          ori_error.resize(3);
          ori_error[0] = -2.0*((ori_error_quat.w()*ori_error_quat.vec()[0] * Kp_ang[0]) + (Kp_ang[2] - Kp_ang[1])*(ori_error_quat.vec()[1]*ori_error_quat.vec()[2]));
          ori_error[1] = -2.0*((ori_error_quat.w()*ori_error_quat.vec()[1] * Kp_ang[1]) + (Kp_ang[0] - Kp_ang[2])*(ori_error_quat.vec()[0]*ori_error_quat.vec()[2]));
          ori_error[2] = -2.0*((ori_error_quat.w()*ori_error_quat.vec()[2] * Kp_ang[2]) + (Kp_ang[1] - Kp_ang[0])*(ori_error_quat.vec()[1]*ori_error_quat.vec()[0]));

          /*---------- computing ang error ----*/
          std::cout << "LhumForiE " << ori_error_quat.vec() << "          " << ori_error_quat.w() << std::endl;
          std::cout << "LhumForiE " << ori_error.array() << std::endl;
          h_error.array() = (des_omega.array() - omega.array());//Lhum inertia.array()*
          std::cout << "Lhumangtau h " << h_error.array() << std::endl;

          angtau.array() = hd_des.array() + ori_error.array();//Lhum + Kd_ang.array() * h_error.array();
          std::cout << "Lhum  " << hd_des << "  !  " << Kd_ang << "   @   " << h_error << "   #   " << ori_error << std::endl;

          std::cout << "Lhum2 " << Kp_ang << std::endl;
          std::cout << "Lhum2 " << (ori_error_quat.w()*ori_error_quat.vec()[0] * Kp_ang[0]) << " % "
                                << (ori_error_quat.w()*ori_error_quat.vec()[1] * Kp_ang[1]) << " ^ "
                                << (ori_error_quat.w()*ori_error_quat.vec()[2] * Kp_ang[2]) << " & " << std::endl;

        }

        else {
          angtau.array() = omega.array() - omega.array();
        }

        /*------------ Safety checks -------*/
        // if (h_error[])

        // cout << ang_tau[1] << endl;
        std::cout << "Lhumangtau " << angtau.array() << std::endl;

        sotDEBUGOUT(15);
        return angtau;
      }




dynamicgraph::Vector& ComImpedanceControl::
  return_end_eff_lqr_tau( dynamicgraph::Vector &end_eff_lqr_tau, int t){

    /** this method computes the desired forces at the end effector based on the
    plan using lqr gains computated with orientation control **/

    end_eff_lqr_tau.resize(12);end_eff_lqr_tau.setZero();

    const dynamicgraph::Vector& lqr_vector = lqrgainSIN(t);
    const dynamicgraph::Vector& position = biasedpositionSIN(t);
    const dynamicgraph::Vector& des_pos = desiredpositionSIN(t);
    const dynamicgraph::Vector& velocity = biasedvelocitySIN(t);
    const dynamicgraph::Vector& des_vel = desiredvelocitySIN(t);
    const dynamicgraph::Vector& des_fff = feedforwardforceSIN(t);
    const dynamicgraph::Vector& des_ori = desoriSIN(t);
    const dynamicgraph::Vector& ori = oriSIN(t);
    const dynamicgraph::Vector& omega = angvelSIN(t);
    const dynamicgraph::Vector& des_omega = desiredangvelSIN(t);



    assert(lqr_vector.size()==156);

    /*------- computing delta X -----------------------------*/
    delta_x.resize(13);
    lqr_pos_error.array() = position.array() - des_pos.array();
    lqr_vel_error.array() = velocity.array() - des_vel.array();
    lqr_ori_error.array() = ori.array() - des_ori.array();
    lqr_ang_vel_error.array() = omega.array() - des_omega.array();

    delta_x << lqr_pos_error, lqr_vel_error, lqr_ori_error, lqr_ang_vel_error;

    /*--------- reshaping the lqr_vector into a 6*13 matrix ------*/

    K.resize(12,13);

    // cout << lqr_vector << endl;

    K.row(0) = lqr_vector.segment(0,13),
    K.row(1) = lqr_vector.segment(13,13),
    K.row(2) = lqr_vector.segment(26,13),
    K.row(3) = lqr_vector.segment(39,13),
    K.row(4) = lqr_vector.segment(52,13),
    K.row(5) = lqr_vector.segment(65,13);
    K.row(6) = lqr_vector.segment(78,13),
    K.row(7) = lqr_vector.segment(91,13),
    K.row(8) = lqr_vector.segment(104,13),
    K.row(9) = lqr_vector.segment(117,13),
    K.row(10) = lqr_vector.segment(130,13),
    K.row(11) = lqr_vector.segment(143,13);

    cout << K << "\n" <<endl;

    /**** computing the centroidal forces at current timestep ----*/
    // F = F* - K*delta_x

    end_eff_lqr_tau = des_fff + K * delta_x;

    sotDEBUGOUT(15);
    return end_eff_lqr_tau;
  }

dynamicgraph::Vector& ComImpedanceControl::
  return_control_torques( dynamicgraph::Vector &tau, int t){
    sotDEBUGIN(15);
    const dynamicgraph::Vector& Kp = KpSIN(t);
    const dynamicgraph::Vector& Kd = KdSIN(t);
    const dynamicgraph::Vector& position = biasedpositionSIN(t);
    const dynamicgraph::Vector& des_pos = desiredpositionSIN(t);
    const dynamicgraph::Vector& velocity = biasedvelocitySIN(t);
    const dynamicgraph::Vector& des_vel = desiredvelocitySIN(t);
    const dynamicgraph::Vector& des_fff = feedforwardforceSIN(t);
    const dynamicgraph::Vector& mass = massSIN(t);
    std::cout << "LhumLqr position " << position.array();
    std::cout << "LhumLqr desposition " << des_pos.array();

    /*----- assertions of sizes -------------*/
    assert(Kp.size() == 3);
    assert(Kd.size() == 3);
    assert(position.size() == 3);
    assert(des_pos.size() == 3);
    assert(velocity.size() == 3);
    assert(des_vel.size() == 3);
    assert(des_fff.size() == 3);

    // cout << "isbiasset" << isbiasset << endl;
    if (isbiasset){
      if(!safetyswitch){
        /*---------- computing position error ----*/
        pos_error.array() = des_pos.array() - position.array();
        vel_error.array() = des_vel.array() - velocity.array();
        /*---------- computing tourques ----*/

        tau.array() = des_fff.array() + (pos_error.array()*Kp.array()
                      + vel_error.array()*Kd.array());//Lhum mass.array()*
      }

    }
    else{
      // quick hack to return zeros

      tau.array() = des_pos.array() - des_pos.array();
    }

    sotDEBUGOUT(15);

    return tau;

  }

dynamicgraph::Vector& ComImpedanceControl::
  compute_end_eff_forces(dynamicgraph::Vector & end_forces, int t){

    sotDEBUGIN(15);

    /**** based on forced distribution from qp *********/

    const dynamicgraph::Vector& lctrl = lctrlSIN(t);
    const dynamicgraph::Vector& actrl = actrlSIN(t);
    const dynamicgraph::Matrix& hess = hessSIN(t);
    const dynamicgraph::Vector& g0 = g0SIN(t);
    const dynamicgraph::Matrix& ce = ceSIN(t);
    const dynamicgraph::Matrix& ci = ciSIN(t);
    const dynamicgraph::Vector& ci0 = ci0SIN(t);
    const dynamicgraph::Matrix& reg = regSIN(t);
    const dynamicgraph::Vector& thrcntvalue = thrcntvalueSIN(t);

    const dynamicgraph::Vector& abs_end_eff_vel = absendeffvelSIN(t);
    std::cout << "L " << lctrl << "A " << actrl << std::endl;

    end_forces.resize(g0.size());
    int dimension = 3;
    int number_of_legs = (g0.size() / dimension - 2) / 2;//number_of_legs * dimension + 2 * dimension +
                                                           //number_of_legs * dimension
    assert(thrcntvalue.size() == number_of_legs);
    int number_of_columns = number_of_legs * dimension + 2 * dimension + number_of_legs * dimension;
    int number_of_identity_columns = 2 * dimension + number_of_legs * dimension;
    if(isbiasset){
      if(thrcntvalue.sum() > 0.5){
        ce0.resize(number_of_identity_columns);
        ce0.setZero();
        // cout << "started " << endl;
        assert(lctrl.size()==3);
        assert(actrl.size()==3);
        assert(g0.size()== number_of_columns);
        assert(ce0.size()==number_of_identity_columns);


        hess_new = hess;
        ce_new = ce;
        ci_new = ci;

        /******* setting up the QP *************************/

        ce0[0] = lctrl[0];
        ce0[1] = lctrl[1];
        ce0[2] = lctrl[2];
        ce0[3] = actrl[0];
        ce0[4] = actrl[1];
        ce0[5] = actrl[2];

        // updating ce_new based on the desired absolute end effector velocity
        if (absendeffposSIN.isPlugged()) {
          const dynamicgraph::Vector& abs_end_eff_pos = absendeffposSIN(t);
          for (int i = 0; i < number_of_legs; i++) {
            double x = abs_end_eff_pos(3 * i);
            double y = abs_end_eff_pos(3 * i + 1);
            double z = abs_end_eff_pos(3 * i + 2); // 0; // Always assumed to be on the floor.
            std::cout << "Lhumz " << i << " " << x << " " << y << " " << z << std::endl;
            ce_new.block<3, 3>(3, 3 * i) << 0, -z,  y,
                                            z,  0, -x,
                                           -y,  x,  0;
          }
        }

        for (int i=0; i < number_of_legs * dimension; i++){
            ce_new(i + 2 * dimension,i) = abs_end_eff_vel(i);
        }


        /******** setting elements of matrix to zero when foot is not
        ********* on the ground ***********************************************/
        float mu = 1.0;
        int condition_number = 5;
        qp.problem(number_of_columns, number_of_identity_columns, number_of_identity_columns);
        for(int i = 0; i < number_of_legs; i++)
          if(thrcntvalue[i] < 0.2){
            // setting the columns related to leg to zero since it is not on the ground
            hess_new.col(0 + dimension * i).setZero();
            hess_new.col(1 + dimension * i).setZero();
            hess_new.col(2 + dimension * i).setZero();

            ce_new.col(0 + dimension * i).setZero();
            ce_new.col(1 + dimension * i).setZero();
            ce_new.col(2 + dimension * i).setZero();
          }
          else{
            std::cout << "Lhum ";
            std::cout << i << endl;
            ci_new(condition_number*i + 0, 3 * i + 0) = 1;
            std::cout << i << endl;
            ci_new(condition_number*i + 0, 3 * i + 2) = -mu;
            std::cout << i << endl;
            ci_new(condition_number*i + 1, 3 * i + 0) = -1;
            std::cout << i << endl;
            ci_new(condition_number*i + 1, 3 * i + 2) = -mu;
            std::cout << i << endl;
            ci_new(condition_number*i + 2, 3 * i + 1) = 1;
            std::cout << i << endl;
            ci_new(condition_number*i + 2, 3 * i + 2) = -mu;
            std::cout << i << endl;
            ci_new(condition_number*i + 3, 3 * i + 1) = -1;
            std::cout << i << endl;
            ci_new(condition_number*i + 3, 3 * i + 2) = -mu;
            std::cout << i << endl;
            ci_new(condition_number*i + 4, 3 * i + 2) = -1;
            std::cout << i << endl;
         }


        // regularizing hessian
        hess_new += reg;
//        cout << "hess5" << hess_new << std::endl;
//        cout << "hess_new: \n" << hess_new << std::endl;
//        cout << "g0: \n" << g0 << std::endl;
//        cout << "ce_new: \n" << ce_new << std::endl;
//        cout << "ce0: \n" << ce0 << std::endl;
//        cout << "ci_new: \n" << ci_new << std::endl;
//        cout << "ci0: \n" << ci0 << std::endl;
        qp.solve(hess_new, g0, ce_new, ce0, ci_new, ci0);
        end_forces = qp.result();

      }
      else{
        // all legs are off the ground. Can not control the COM
        end_forces.setZero();
      }
    }

    else {
      // cout << "not set" << endl;
      end_forces.setZero();
    }

     cout << "end_forces " << end_forces.array() << endl;
    return end_forces;

  }

dynamicgraph::Vector& ComImpedanceControl::
  compute_des_com_pos(dynamicgraph::Vector& des_com_pos, int t){
    // this computes the desired com position to balance a planck with force control

    sotDEBUGIN(15);

    const dynamicgraph::Vector& leg_length_fl = leglengthflSIN(t);
    const dynamicgraph::Vector& leg_length_hl = leglengthhlSIN(t);

    assert(leg_length_fl.size() == 6);
    assert(leg_length_hl.size() == 6);

    des_com_pos.resize(3);
    des_com_pos[1] = 0;
    des_com_pos[2] = 0.2;

    diff.resize(3);
    diff[2] = leg_length_fl[2] - leg_length_hl[2];
    diff[0] = leg_length_fl[0] - leg_length_hl[0] + 0.42;
    diff[1] = 0;

    des_com_pos[0] = 0.1*atan(diff[2]/diff[0]);

    // cout << des_com_pos[0] << endl;

    sotDEBUGOUT(15);
    return des_com_pos;
  }


dynamicgraph::Vector& ComImpedanceControl::
  set_pos_bias(dynamicgraph::Vector& pos_bias, int t){
    // NOTE : when a value of the signal is not computed at all timesteps
    // create another variable tp compute the value and assign it to the signal
    // at all time steps to ensure that the value of the signal is constant.
    // When not done, the signal value fluctuates

    sotDEBUGIN(15);
    const dynamicgraph::Vector& position = positionSIN(t);

    position_bias.resize(3);
    pos_bias.resize(3);

    if(init_flag_pos){
      t_start = t;
      position_bias.setZero();
      pos_bias.setZero();
      init_flag_pos = 0;
    }

    if (isbiasset){
      // cout << "bias set" << endl;
      for(int j = 0; j < 3 ; j++){
        pos_bias[j] = position_bias[j];
      }
    }
    else{
      isbiasset = 1;
      for(int i = 0 ; i < 3; i ++){
        position_bias[i] = position[i];
        pos_bias[i] = position_bias[i];
      }
    }

    pos_bias[2] = 0.0;

    sotDEBUGOUT(15);
    return pos_bias;

  }


dynamicgraph::Vector& ComImpedanceControl::
  set_vel_bias(dynamicgraph::Vector& vel_bias, int t){
    sotDEBUGIN(15);
    const dynamicgraph::Vector& velocity = velocitySIN(t);


    velocity_bias.resize(3);
    vel_bias.resize(3);

    assert(velocity.size()==3);

    if(init_flag_vel){
      t_start = t;
      velocity_bias.setZero();
      vel_bias.setZero();
      init_flag_vel = 0;
    }

    if (isbiasset){
      // cout << "bias set" << endl;
      for(int j = 0; j < 3 ; j++){
        vel_bias[j] = velocity_bias[j];
      }
    }
    else{
      isbiasset = 1;
      for(int i = 0 ; i < 3; i ++){
        velocity_bias[i] += velocity[i];
        vel_bias[i] = velocity_bias[i];
      }
    }

    sotDEBUGOUT(15);
    return vel_bias;

  }

dynamicgraph::Vector& ComImpedanceControl::
  threshold_cnt_sensor(dynamicgraph::Vector& thr_cnt_sensor, int t){//TODO Lhum
    // This thresholds the values of the contact sensors
    // less than 0.2 is set to zero, and greater than 0.8 is set to 1
    // This is neccessary because the cnt_sensor is noisy at the extremes

    sotDEBUGIN(15);
    const dynamicgraph::Vector& cnt_sensor = cntsensorSIN(t);

    thr_cnt_sensor.resize(4); thr_cnt_sensor.fill(0.);

    for(int i = 0; i < 4; i++){
      if (cnt_sensor[i] < 0.20){
        thr_cnt_sensor[i] = 0.0 ;
      }
      // else if (cnt_sensor[i] > 0.8){
      //   thr_cnt_sensor[i] = 1.0;
      // }
      else{
        thr_cnt_sensor[i] = 1.0;
      }
      // cnt sensor return 0 when in contact
      // negating it to one
      thr_cnt_sensor[i] = 1 - thr_cnt_sensor[i];
    }
    sotDEBUGOUT(15);

    return thr_cnt_sensor;
  }
