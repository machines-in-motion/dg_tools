"""
@package dg_blmc_robots
@file leg_impedance_controller.py
@author Avadesh Meduri
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-02-06
@brief This code contains implementation of leg impedance impedance_controller
"""

#################### Imports #################################################

from dg_tools.utils import *
from dg_tools.traj_generators import mul_double_vec_2

from py_robot_properties_teststand.config import TeststandConfig
import dynamic_graph.sot.dynamic_pinocchio as dp
from dynamic_graph.sot.core.switch import SwitchVector


#############################################################################

class LegImpedanceController():
    def __init__(self, leg_name):
        self.leg_name = leg_name
        self.robot_pin = TeststandConfig.buildRobotWrapper()
        self.robot_dg = dp.DynamicPinocchio(self.leg_name)
        self.robot_dg.setModel(self.robot_pin.model)
        self.robot_dg.setData(self.robot_pin.data)

        self.robot_dg.createJacobianEndEffWorld(
            'jac_cnt_' + self.leg_name, 'contact')
        self.robot_dg.createPosition('pos_hip_' + self.leg_name, 'HFE')
        self.robot_dg.createPosition('pos_foot_' + self.leg_name, 'END')


        print("Warning: Robot acceleration has been set to zero")
        self.robot_dg.acceleration.value = 3 * (0.0, )

        self.joint_positions = Selec_of_vector("joint_position")
        self.joint_positions.selec(1, 3)
        plug(self.robot_dg.position, self.joint_positions.sin)
        self.joint_velocities = Selec_of_vector("joint_velocity")
        self.joint_velocities.selec(1, 3)
        plug(self.robot_dg.velocity, self.joint_velocities.sin)
        self.ati = Multiply_of_vector("ATI")
        plug(constVector([1.0, 1.0, 1.0, 1.0, 1.0, 1.0], "one6"), self.ati.sin0)

    def _compute_control_torques(self, RNEADM, start_index=1, end_index=3):
        '''
        ## computes torques tau = JacT*(errors)
        ## Input : start_index (to select actuated torques to plug ot the robot)
                 : end_index (to select actuated torques to plug to the robot)
        '''
        jacT = transpose_mat(self.jac, "jacTranspose" + self.leg_name)
        # multiplying negative
        errors = mul_double_vec(-1.0, self.total_error_with_force,
                                "neg_op_" + self.leg_name)###########self.total_error
        control_torques = multiply_mat_vec(
            jacT, errors, "compute_control_torques_" + self.leg_name)


        self.friction = mul_double_vec(-0.00001341 * 81 * 0, self.robot_dg.velocity, "friction")

        self.v_negative_one = constVector([-1.0,], "v_negative_one")
        self.v_one = constVector([1.0, ], "v_one")
        self.v_zero = constVector([0.0, ], "v_zero")

        ### Setup the switches
        self.sel_v1 = Selec_of_vector("sel_v1" + self.leg_name)
        self.sel_v1.selec(1, 2)
        plug(self.robot_dg.velocity, self.sel_v1.sin)

        self.compare1 = CompareVector("compare1")
        plug(self.v_zero, self.compare1.sin1)
        plug(self.sel_v1.sout, self.compare1.sin2)


        self.v_negative_vel1 = mul_vec_vec(self.v_negative_one, self.sel_v1.sout, "negative_vel1")

        self.switch1 = SwitchVector("switch1")
        self.switch1.setSignalNumber(2)
        plug(self.v_one, self.switch1.sin0)
        plug(self.v_negative_one, self.switch1.sin1)
        plug(self.compare1.sout, self.switch1.boolSelection)

        self.sel_v2 = Selec_of_vector("sel_v2" + self.leg_name)
        self.sel_v2.selec(2, 3)
        plug(self.robot_dg.velocity, self.sel_v2.sin)

        self.compare2 = CompareVector("compare2")
        plug(self.v_zero, self.compare2.sin1)
        plug(self.sel_v2.sout, self.compare2.sin2)

        self.v_negative_vel2 = mul_vec_vec(self.v_negative_one, self.sel_v2.sout, "negative_vel2")


        self.switch2 = SwitchVector("switch1")
        self.switch2.setSignalNumber(2)
        plug(self.v_one, self.switch2.sin0)
        plug(self.v_negative_one, self.switch2.sin1)
        plug(self.compare2.sout, self.switch2.boolSelection)

        self.semi_vel = stack_two_vectors(self.switch1.sout, self.switch2.sout, 1, 1)#, "semi1")
        self.vel = stack_two_vectors(constVector([0.0, ], "zero1"), self.semi_vel, 1, 2)#, "vel")

        self.friction2 = mul_double_vec(0.00540994 * 9, self.vel, "friction2")

        self.newjac = Selec_of_matrix("new_jac")
        self.newjac.selecRows(0, 3)
        self.newjac.selecCols(0, 3)
        plug(self.jac, self.newjac.sin)

        self.jacT = transpose_mat(self.jac, "jacT")
        self.jacTI = Inverse_of_matrix("jacTI")
        plug(self.jacT, self.jacTI.sin)

        # c1 = constVector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "c1")
        # c2 = constVector([0.0, 1.0, 0.0, 0.0, 0.0, 0.0], "c2")
        # c3 = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "c3")
        # mc1 = multiply_mat_vec(self.jacT, c1, "mc1")
        # mc2 = multiply_mat_vec(self.jacT, c2, "mc2")
        # mc3 = multiply_mat_vec(self.jacT, c3, "mc3")

        self.final2 = multiply_mat_vec(self.jacTI.sout, self.friction2, "final2")

        ## selecting the torques to be plugged to the robot
        sel = Selec_of_vector("impedance_torques_" + self.leg_name)
        sel.selec(start_index - 1, end_index)
        plug(control_torques, sel.signal('sin'))

        sel2 = Selec_of_vector("sel_torque_" + self.leg_name)
        sel2.selec(start_index, end_index)
        plug(add_vec_vec(sel.sout, self.friction, "torque"), sel2.sin)
        self.final3 = add_vec_vec(sel2.sout, RNEADM, "final3")
        return self.final3

    def return_leg_length(self):
        '''
        ### Computes current leg length
        '''

        self.xyzpos_hip = hom2pos(self.robot_dg.signal(
            "pos_hip_" + self.leg_name), "xyzpos_hip_" + self.leg_name)
        self.xyzpos_foot = hom2pos(self.robot_dg.signal(
            "pos_foot_" + self.leg_name), "xyzpos_foot_" + self.leg_name)

        # Relative position of foot with respect to the base of the foot
        self.rel_pos_foot = subtract_vec_vec(
            self.xyzpos_foot, self.xyzpos_hip, "rel_pos_foot_" + self.leg_name)
        self.rel_pos_foot = stack_two_vectors(self.rel_pos_foot, constVector(
            [0.0, 0.0, 0.0], 'stack_to_wrench_' + self.leg_name), 3, 3)
        return self.rel_pos_foot

    def return_control_torques(self, kp, des_pos, kd=None, des_vel=None, kf=None, fff=None, kfe=None, RNEADM=None):
        '''
        ## Impedance controller implementation. Creates a virtual spring
        ## damper system the base and foot of the leg
        ## Input : Kp - proportional gain
                 : Kd - derivative gain
                 : des_pos - desired position (size : 1*6 )
                 : des_vel - desired velocity (size : 1*6 )
                 : Kf - feed forward force gain (safety)
                 : fff - feed forward force (size : 1*6)
        '''
        self.rct_args = dict()
        self.rct_args["kp"] = kp
        self.rct_args["des_pos"] = des_pos
        self.rct_args["kd"] = kd
        self.rct_args["des_vel"] = des_vel
        self.rct_args["kf"] = kf
        self.rct_args["fff"] = fff

        self.jac = self.robot_dg.signal("jac_cnt_" + self.leg_name)

        self.return_leg_length()
        self.pos_error = subtract_vec_vec(
            self.rel_pos_foot, des_pos, "pos_error_" + self.leg_name)
        mul_kp_gains_split = Multiply_of_vector("kp_split_" + self.leg_name)
        plug(kp, mul_kp_gains_split.sin0)
        plug(self.pos_error, mul_kp_gains_split.sin1)
        pos_error_with_gains = mul_kp_gains_split.sout

        if kd is not None and des_vel is not None:
            print("Kd !!!!!")
            self.rel_vel_foot = multiply_mat_vec(
                self.jac, self.robot_dg.velocity, "rel_vel_foot_" + self.leg_name)
            self.vel_error = subtract_vec_vec(
                self.rel_vel_foot, des_vel, "vel_error_" + self.leg_name)
            mul_kd_gains_split = Multiply_of_vector(
                "kd_split_" + self.leg_name)
            plug(kd, mul_kd_gains_split.sin0)
            plug(self.vel_error, mul_kd_gains_split.sin1)
            vel_error_with_gains = mul_kd_gains_split.sout

            self.pd_error = add_vec_vec(
                pos_error_with_gains, vel_error_with_gains, "pd_error_" + self.leg_name)

        else:
            self.pd_error = pos_error_with_gains

        if kf is not None and fff is not None:
            print("fff is plugged ....")
            mul_kf_gains = Multiply_double_vector("Kf_" + self.leg_name)
            plug(kf, mul_kf_gains.sin1)
            plug(fff, mul_kf_gains.sin2)
            fff_with_gains = mul_kf_gains.sout
            self.total_error = add_vec_vec(
                fff_with_gains, self.pd_error, "total_error_" + self.leg_name)

            self.estimated_foot_force = add_vec_vec(
                pos_error_with_gains, fff_with_gains, "est_f_" + self.leg_name)

        else:
            self.total_error = self.pd_error

            self.estimated_foot_force = add_vec_vec(pos_error_with_gains, zero_vec(
                6, "zero_vec_est_f" + self.leg_name), "est_f_" + self.leg_name)
        mul_kfe = mul_vec_vec(kfe, subtract_vec_vec(self.total_error, self.ati.sout, "force_error_" + self.leg_name), "Kfe_" + self.leg_name)
        #it is as easy as change self.total_error to fff:)
        self.total_error_with_force = add_vec_vec(self.total_error, mul_kfe, "total_error_with_force_" + self.leg_name)
        control_torques = self._compute_control_torques(RNEADM)

        return control_torques

    def return_joint_ctrl_torques(self, kp_joint, des_joint_pos, kd_joint, des_joint_vel):
        pos_error = subtract_vec_vec(des_joint_pos, self.joint_positions.sout, "joint_pos_error")
        vel_error = subtract_vec_vec(des_joint_vel, self.joint_velocities.sout, "joint_vel_error")
        mul_kp_gains = Multiply_of_vector("mul_kp_joint")
        plug(kp_joint, mul_kp_gains.sin0)
        plug(pos_error, mul_kp_gains.sin1)
        pos_error_with_gains = mul_kp_gains.sout
        mul_kd_gains = Multiply_of_vector("mul_kd_joint")
        plug(kd_joint, mul_kd_gains.sin0)
        plug(vel_error, mul_kd_gains.sin1)
        vel_error_with_gains = mul_kd_gains.sout
        jtorque = add_vec_vec(pos_error_with_gains, vel_error_with_gains, "joint_torque")
        return jtorque

    def record_data(self, robot):

        robot.add_trace("rel_pos_foot_" + self.leg_name, "sout")
        # robot.add_ros_and_trace("rel_pos_foot_" + self.leg_name, "sout")

        robot.add_trace("rel_vel_foot_" + self.leg_name, "sout")
        # robot.add_ros_and_trace("rel_vel_foot_" + self.leg_name, "sout")

        robot.add_trace("pos_error_" + self.leg_name, "sout")
        # robot.add_ros_and_trace("pos_error_" + self.leg_name, "sout")

        robot.add_trace("vel_error_" + self.leg_name, "sout")
        # robot.add_ros_and_trace("vel_error_" + self.leg_name, "sout")

        robot.add_trace("est_f_" + self.leg_name, "sout")
        # robot.add_ros_and_trace("est_f_" + self.leg_name, "sout")

        if (self.rct_args["kf"] is not None and
                self.rct_args["fff"] is not None):
            robot.add_trace("total_error_" + self.leg_name, "sout")
            # robot.add_ros_and_trace("total_error_" + self.leg_name, "sout")