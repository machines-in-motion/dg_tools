"""
@package dg_blmc_robots
@file solo12_impedance_controller.py
@author Julian Viereck
@author Avadesh Meduri
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-11-27
@brief Implement leg impedance controller and 4 leg control for solo12.

Differences between solo8 and solo12 code:
- Using 3-dim vector to specify desired position and velocity instead of 6-dim
- Use kp and kd value for each leg separately
"""

from robot_properties_solo.config import Solo12Config
import dynamic_graph.sot.dynamics_pinocchio as dp
import dynamic_graph as dg
from dg_tools.utils import *

from dg_tools.leg_impedance_control.quad_leg_impedance_controller import QuadrupedComControl

import dynamic_graph.sot.dynamics_pinocchio as dp

from dynamic_graph.sot.core.math_small_entities import (
    Selec_of_matrix
)

class Solo12ComController(QuadrupedComControl):
    def init_robot_properties(self):
        self.robot_vicon_name = "solo12"
        self.robot_mass = 3 * [Solo12Config.mass]

        # TODO: Provide the new base inertia here.
        self.robot_base_inertia = [0.00578574, 0.01938108, 0.02476124]

    def track_com(self):
        """Instead of tracking the biased base position and velocity, track the
        biased com and vcom position.
        """
        # Init a dg pinocchio robot here for com and vcom computation.
        self.robot_pin = Solo12Config.buildRobotWrapper()
        self.robot_dg = dp.DynamicPinocchio('')
        self.robot_dg.setModel(self.robot_pin.model)
        self.robot_dg.setData(self.robot_pin.data)

        self.robot_position = stack_two_vectors(
            self.get_biased_base_position(),
            self.robot.device.joint_positions, 6, 12)
        self.robot_velocity = stack_two_vectors(
            self.get_biased_base_velocity(),
            self.robot.device.joint_velocities, 6, 12)

        dg.plug(self.robot_position, self.robot_dg.position)
        dg.plug(self.robot_velocity, self.robot_dg.velocity)
        self.robot_dg.acceleration.value = 18 * [0.,]

        # These are used in the reactive_stepper.
        self.robot_com = self.robot_dg.com
        self.robot_vcom = multiply_mat_vec(
            self.robot_dg.Jcom, self.robot_velocity)

        dg.plug(self.robot_com, self.com_imp_ctrl.biased_pos)
        dg.plug(self.robot_vcom, self.com_imp_ctrl.biased_vel)


    def return_com_torques(self, *args, **kwargs):
        """
        @return 12-dim vector with forces at each endeffector.
        """
        super(Solo12ComController, self).return_com_torques(*args, **kwargs)
        return self.wb_ctrl


class Solo12LegImpedanceController(object):
    """Impedance controller for single leg on solo12."""
    def __init__(self, leg_name, joint_indices_range):
        """
        Args:
            leg_name: (str) Name of the leg, like "FL"
            joint_indices_range: (array) Indices on the joint array of this leg.
        """
        self.leg_name = leg_name
        self.joint_indices_range = joint_indices_range

        self.robot_pin = Solo12Config.buildRobotWrapper()
        self.robot_dg = dp.DynamicPinocchio(self.leg_name)
        self.robot_dg.setModel(self.robot_pin.model)
        self.robot_dg.setData(self.robot_pin.data)

        self.robot_dg.createJacobianEndEffWorld(
            'jac_cnt_' + self.leg_name, self.leg_name + '_ANKLE')
        self.robot_dg.createPosition('pos_hip_' + self.leg_name, self.leg_name + '_HFE')
        self.robot_dg.createPosition('pos_foot_' + self.leg_name, self.leg_name + '_ANKLE')

        self.robot_dg.acceleration.value = 18 * (0.0, )
        self.joint_positions_sin = self.robot_dg.position
        self.joint_velocities_sin = self.robot_dg.velocity

    def _compute_leg_length(self):
        """ Computes current leg length. """
        self.xyzpos_hip = hom2pos(self.robot_dg.signal(
            "pos_hip_" + self.leg_name), "xyzpos_hip_" + self.leg_name)
        self.xyzpos_foot = hom2pos(self.robot_dg.signal(
            "pos_foot_" + self.leg_name), "xyzpos_foot_" + self.leg_name)

        # Relative position of foot with respect to the base of the foot
        self.rel_pos_foot = subtract_vec_vec(
            self.xyzpos_foot, self.xyzpos_hip, "rel_pos_foot_" + self.leg_name)
        return self.rel_pos_foot

    def _compute_jacobian(self):
        """ Slice the jacobian to keep only the parts needed for this leg.

        Returns: Jacobian at the endeffector, size 6 x 3 matrix.
        """
        jac_sub = Selec_of_matrix('jac_cnt_' + self.leg_name)
        jac_sub.selecRows(0, 3)
        jac_sub.selecCols(6 + self.joint_indices_range[0], 6 + self.joint_indices_range[1])
        dg.plug(self.robot_dg.signal("jac_cnt_" + self.leg_name), jac_sub.sin)
        return jac_sub.sout

    def _compute_control_torques(self):
        """
        Computes torques tau = JacT*(errors).
        """
        self.jacT = transpose_mat(self.jac, "jacTranspose" + self.leg_name)
        # multiplying negative
        errors = mul_double_vec(-1.0, self.total_error,
                                "neg_op_" + self.leg_name)
        control_torques = multiply_mat_vec(
            self.jacT, errors, "compute_control_torques_" + self.leg_name)
        return control_torques

    def compute_control_torques(self, kp, kd, kf, des_pos, des_vel, fff, pos_global=False):
        """
        Impedance controller implementation. Creates a virtual spring
        damper system the base and foot of the leg

        Args
            Kp: proportional gain
            Kd: derivative gain
            des_pos: desired position (size : 1*3 )
            des_vel: desired velocity (size : 1*3 )
            Kf: feed forward force gain (safety)
            fff: feed forward force (size : 1*)
            pos_global: If true, assume the des_pos is in global world coordinates.
               Otherwise, use local coordinate system of the
        Returns:

        """
        self.rct_args = dict()
        self.rct_args["kp"] = kp
        self.rct_args["des_pos"] = des_pos
        self.rct_args["kd"] = kd
        self.rct_args["des_vel"] = des_vel
        self.rct_args["kf"] = kf
        self.rct_args["fff"] = fff
        self.rct_args["pos_global"] = pos_global

        self.jac = self._compute_jacobian()
        self.rel_pos_foot = self._compute_leg_length()

        if pos_global:
            self.pos_error = subtract_vec_vec(
                self.xyzpos_foot, des_pos, "pos_error_" + self.leg_name)
        else:
            self.pos_error = subtract_vec_vec(
                self.rel_pos_foot, des_pos, "pos_error_" + self.leg_name)
        mul_kp_gains_split = Multiply_of_vector("kp_split_" + self.leg_name)
        plug(kp, mul_kp_gains_split.sin0)
        plug(self.pos_error, mul_kp_gains_split.sin1)
        pos_error_with_gains = mul_kp_gains_split.sout

        if kd is not None and des_vel is not None:
            print("Kd !!!!!")
            leg_joint_dq = selec_vector(self.robot_dg.velocity,
                6 + self.joint_indices_range[0], 6 + self.joint_indices_range[1], 'dq_' + self.leg_name)
            self.rel_vel_foot = multiply_mat_vec(
                self.jac, leg_joint_dq, "rel_vel_foot_" + self.leg_name)
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

        self.control_torques = self._compute_control_torques()
        return self.control_torques

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

class Solo12ImpedanceController(object):
    """ Implements leg impedance controller for the solo12 robot.

    Compared to the solo8 implementation, this uses the full solo12 urdf.
    Especially, it computes the desired torque at the actual legs instead of
    using the same "teststand" leg at each joint.
    """
    def __init__(self, robot):
        self.robot = robot
        self.leg_imp_ctrl = []
        self.leg_idx = [[0, 3], [3, 6], [6, 9], [9, 12]]
        for leg_name, leg_idx in zip(['FL', 'FR', 'HL', 'HR'], self.leg_idx):
            self.leg_imp_ctrl.append(
                Solo12LegImpedanceController(leg_name, leg_idx))

        self.abs_end_eff_pos = None
        self.com_end_eff_pos = None

    def _compute_leg_control_torques(self, leg_idx, kp, kd, kf, des_pos, des_vel, fff):
        """
        """
        self.leg_imp_contrl

    def _slice_vec(self, vec, leg_idx, name):
        """ Helper for slicing the desired vector for a leg.

        Args:
            vec: (1*12 vector or None) Vector to slice for a given leg
            leg_idx: (int) Leg index to slice the vector for
            name: (str) Name for the slicing operator to use.

        Returns:
            1*3 vector slice for the leg, None if vec was None.
        """
        if vec is None:
            return None
        else:
            return selec_vector(vec, 3 * leg_idx, 3 * leg_idx + 3, name)

    def compute_control_torques(self, kp, des_pos, kd=None, des_vel=None, kf=None, fff=None,
                                base_position=None, base_velocity=None, pos_global=False):
        """ Computes the desired joint torques for desired configuration using impedance controller.

        If no base position or velocity is provided, assume the base if fixed at the origin.

        Args:
            Kp: (double) proportional gain (double)
            des_pos: (1*12 vector) desired_position in current time step
            Kd: derivative gain (double)
            des_vel: (1*12 vector) desired_velocity in current time step
            fff: (1*12 vector) Feed forward force
            base_position: (1*7 vector, optional) Base position (translation + quaternion)
            base_velocity: (1*6 vector, optional) Base velocity (translation + rotation)
            pos_global: If true, track des_pos in global frame.
                Requires base_position and base_velocity to be set.
        Returns:
            Final joint torques (1 * 12 vector)
        """
        # self.robot = robot
        self.des_pos = vectorIdentity(des_pos, 12, "imp_ctrl_des_pos")
        self.des_vel = vectorIdentity(des_vel, 12, "imp_ctrl_des_vel")
        self.fff = vectorIdentity(fff, 12, "imp_ctrl_fff")
        self.joint_positions = self.robot.device.joint_positions
        self.joint_velocities = self.robot.device.joint_velocities

        # If no base information is provided, assume the base is fixed.
        if base_position is None or base_velocity is None:
            base_position = constVector([0, 0, 0, 0, 0, 0, 0])
            base_velocity = constVector([0, 0, 0, 0, 0, 0])

        # Convert the base_position into PoseRPY
        base_pose_rpy = basePoseQuat2PoseRPY(base_position)

        self.robot_position = stack_two_vectors(base_pose_rpy, self.joint_positions, 6, 12)
        self.robot_velocity = stack_two_vectors(base_velocity, self.joint_velocities, 6, 12)

        for leg_idx, imp_controller in enumerate(self.leg_imp_ctrl):
            leg_name = imp_controller.leg_name
            dg.plug(self.robot_position, imp_controller.robot_dg.position)
            dg.plug(self.robot_velocity, imp_controller.robot_dg.velocity)

            # Plug the desired position, velocity and forces.
            imp_controller.compute_control_torques(
                self._slice_vec(kp, leg_idx, 'kp_' + leg_name),
                self._slice_vec(kd, leg_idx, 'kd_' + leg_name),
                kf,
                self._slice_vec(self.des_pos, leg_idx, 'des_pos_slice_' + leg_name),
                self._slice_vec(self.des_vel, leg_idx, 'des_vel_slice_' + leg_name),
                self._slice_vec(fff, leg_idx, 'fff_slice_' + leg_name),
                pos_global=pos_global)

        # Combine the computed torques from the impedance controllers into single torque vector.
        self.control_torques = stack_two_vectors(
            stack_two_vectors(
                self.leg_imp_ctrl[0].control_torques,
                self.leg_imp_ctrl[1].control_torques, 3, 3),
            stack_two_vectors(
                self.leg_imp_ctrl[2].control_torques,
                self.leg_imp_ctrl[3].control_torques, 3, 3),
            6, 6
        )

        return self.control_torques

    def compute_com_end_eff_pos(self):
        """Returns the endeffector positions wrt com position in world frame.

        Returns:
          Signal<dg::Vector, size=12> Stack of the four endeffector positions
        """
        if self.com_end_eff_pos:
            return self.com_end_eff_pos

        com_signal = self.leg_imp_ctrl[0].robot_dg.com

        self.com_end_eff_pos = stack_two_vectors(
            stack_two_vectors(
                subtract_vec_vec(self.leg_imp_ctrl[0].xyzpos_foot, com_signal),
                subtract_vec_vec(self.leg_imp_ctrl[1].xyzpos_foot, com_signal),
                3, 3),
            stack_two_vectors(
                subtract_vec_vec(self.leg_imp_ctrl[2].xyzpos_foot, com_signal),
                subtract_vec_vec(self.leg_imp_ctrl[3].xyzpos_foot, com_signal),
                3, 3),
            6, 6, "imp_ctrl_com_end_eff_pos"
        )

        return self.com_end_eff_pos

    def compute_abs_end_eff_pos(self):
        """Returns the endeffector positions wrt world frame.

        Returns:
          Signal<dg::Vector, size=12> Stack of the four endeffector positions
        """
        if self.abs_end_eff_pos:
            return self.abs_end_eff_pos

        com_signal = self.leg_imp_ctrl[0].robot_dg.com

        self.abs_end_eff_pos = stack_two_vectors(
            stack_two_vectors(
                self.leg_imp_ctrl[0].xyzpos_foot,
                self.leg_imp_ctrl[1].xyzpos_foot,
                3, 3),
            stack_two_vectors(
                self.leg_imp_ctrl[2].xyzpos_foot,
                self.leg_imp_ctrl[3].xyzpos_foot,
                3, 3),
            6, 6, "imp_ctrl_abs_end_eff_pos"
        )

        return self.abs_end_eff_pos

    def record_data(self):
        for imp_controller in self.leg_imp_ctrl:
            imp_controller.record_data(self.robot)

        self.compute_abs_end_eff_pos()

        self.robot.add_trace("imp_ctrl_des_pos", "sout")
        self.robot.add_trace("imp_ctrl_des_vel", "sout")
        self.robot.add_trace("imp_ctrl_fff", "sout")
        self.robot.add_trace("imp_ctrl_abs_end_eff_pos", "sout")
