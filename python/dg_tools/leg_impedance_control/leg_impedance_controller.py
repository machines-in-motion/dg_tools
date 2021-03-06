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

from robot_properties_teststand.config import TeststandConfig
import dynamic_graph.sot.dynamic_pinocchio as dp


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
        self.robot_dg.add_signals()

        print("Warning: Robot acceleration has been set to zero")
        self.robot_dg.acceleration.value = np.array(3* [0.0])

        self.joint_positions_sin = self.robot_dg.position
        self.joint_velocities_sin = self.robot_dg.velocity

    def _compute_control_torques(self, start_index=1, end_index=3):
        '''
        ## computes torques tau = JacT*(errors)
        ## Input : start_index (to select actuated torques to plug ot the robot)
                 : end_index (to select actuated torques to plug to the robot)
        '''
        jacT = transpose_mat(self.jac, "jacTranspose" + self.leg_name)
        # multiplying negative
        errors = mul_double_vec(-1.0, self.total_error,
                                "neg_op_" + self.leg_name)
        control_torques = multiply_mat_vec(
            jacT, errors, "compute_control_torques_" + self.leg_name)

        # selecting the torques to be plugged to the robot
        sel = Selec_of_vector("impedance_torques_" + self.leg_name)
        sel.selec(start_index, end_index)
        plug(control_torques, sel.signal('sin'))
        return sel.signal('sout')

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
            np.array([0.0, 0.0, 0.0]), 'stack_to_wrench_' + self.leg_name), 3, 3)
        return self.rel_pos_foot

    def return_control_torques(self, kp, des_pos, kd=None, des_vel=None, kf=None, fff=None):
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
        plug(kp, mul_kp_gains_split.sin(0))
        plug(self.pos_error, mul_kp_gains_split.sin(1))
        pos_error_with_gains = mul_kp_gains_split.sout

        if kd is not None and des_vel is not None:
            print("Kd !!!!!")
            self.rel_vel_foot = multiply_mat_vec(
                self.jac, self.robot_dg.velocity, "rel_vel_foot_" + self.leg_name)
            self.vel_error = subtract_vec_vec(
                self.rel_vel_foot, des_vel, "vel_error_" + self.leg_name)
            mul_kd_gains_split = Multiply_of_vector(
                "kd_split_" + self.leg_name)
            plug(kd, mul_kd_gains_split.sin(0))
            plug(self.vel_error, mul_kd_gains_split.sin(1))
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

        control_torques = self._compute_control_torques()

        return control_torques

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
