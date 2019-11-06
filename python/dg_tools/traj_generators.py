"""
@package dg_blmc_robots
@file traj_generators.py
@author Avadesh Meduri
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-02-06
@brief This code contains different trajectory generation functions
"""

import numpy as np
from numpy import math
import dynamic_graph.sot.tools
import dynamic_graph as dg

from .utils import *
from .math_small_entities import *


#########################################################################


def add_doub_doub_2(db1, db2, entityName):
    add = Add_of_double(entityName)
    plug(db1, add.signal('sin1'))
    plug(db2, add.signal('sin2'))
    return add.sout


def mul_double_vec_2(doub, vec, entityName):
    # need double as a signal
    mul = Multiply_double_vector(entityName)
    plug(doub, mul.signal('sin1'))
    plug(vec, mul.signal('sin2'))
    return mul.sout


def scale_values(double, scale, entityName):
    mul = Multiply_of_double(entityName)
    mul.sin0.value = scale
    plug(double, mul.sin1)
    return mul.sout


def mul_doub_doub(db1, db2, entityName):
    dif1 = Multiply_of_double(entityName)
    plug(db1, dif1.sin0)
    plug(db2, dif1.sin1)
    return dif1.sout

##########################################################


# For making gain input dynamic through terminal
add_pi = Add_of_double('pi')
add_pi.sin1.value = 0
# Change this value for different gains
add_pi.sin2.value = np.pi/2.0
pi = add_pi.sout

###############################################################################


def sine_generator(amplitude, omega, phase, bias, entityName):
    # generates a y = a*sin(W.t + phi)
    osc_pos = dynamic_graph.sot.tools.Oscillator(entityName + "_pos")
    osc_pos.setTimePeriod(0.001)
    plug(omega, osc_pos.omega)
    plug(amplitude, osc_pos.magnitude)
    plug(phase, osc_pos.phase)
    # osc_pos.phase.value = phase
    osc_pos.bias.value = bias

    osc_vel = dynamic_graph.sot.tools.Oscillator(entityName + '_vel')
    osc_vel.setTimePeriod(0.001)
    plug(osc_pos.omega, osc_vel.omega)
    plug(osc_pos.magnitude, osc_vel.magnitude)
    plug(add_doub_doub_2(osc_pos.phase, pi, entityName + "phase_add"), osc_vel.phase)
    osc_vel.bias.value = 0

    unit_vector_pos = constVector(
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], entityName + "_unit_vector_pos")
    unit_vector_vel = constVector(
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], entityName + "_unit_vector_vel")

    pos_traj = mul_double_vec_2(
        osc_pos.sout, unit_vector_pos, entityName + "_des_position")
    vel_traj = mul_double_vec_2(
        osc_vel.sout, unit_vector_vel, entityName + "_des_velocity")

    return pos_traj, vel_traj


def circular_trajectory_generator(radius_x, radius_z, omega,
                                  phase, bias, entityName):
    """
    generates a circular circular_trajectory_generator
    """
    ## Position################################################################

    osc_x = dynamic_graph.sot.tools.Oscillator(entityName + "_posx")
    osc_x.setTimePeriod(0.001)
    plug(omega, osc_x.omega)
    plug(radius_x, osc_x.magnitude)
    plug(phase, osc_x.phase)
    osc_x.bias.value = 0.0

    osc_xd = dynamic_graph.sot.tools.Oscillator(entityName + '_velx')
    osc_xd.setTimePeriod(0.001)
    plug(osc_x.omega, osc_xd.omega)
    plug(mul_doub_doub(osc_x.omega, osc_x.magnitude, "dx"), osc_xd.magnitude)
    #plug(osc_x.magnitude, osc_xd.magnitude)
    plug(add_doub_doub_2(osc_x.phase, pi, entityName + "phase_addx"), osc_xd.phase)
    osc_xd.bias.value = 0

    osc_z = dynamic_graph.sot.tools.Oscillator(entityName + "_posz")
    osc_z.setTimePeriod(0.001)
    plug(omega, osc_z.omega)
    plug(radius_z, osc_z.magnitude)
    plug(add_doub_doub_2(osc_x.phase, pi, entityName + "phase_addx"), osc_z.phase)
    osc_z.bias.value = bias

    osc_zd = dynamic_graph.sot.tools.Oscillator(entityName + '_velz')
    osc_zd.setTimePeriod(0.001)
    plug(osc_z.omega, osc_zd.omega)
    plug(mul_doub_doub(osc_z.omega, osc_z.magnitude, "dz"), osc_zd.magnitude)
    # plug(osc_z.magnitude, osc_zd.magnitude)
    plug(add_doub_doub_2(osc_z.phase, pi, entityName + "phase_addz"), osc_zd.phase)
    osc_zd.bias.value = 0

    unit_vector_x = constVector(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], entityName + "unit_vector_x")
    unit_vector_z = constVector(
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], entityName + "unit_vector_z")

    pos_des_x = mul_double_vec_2(
        osc_x.sout, unit_vector_x, entityName + "sine_des_position_x")
    pos_des_z = mul_double_vec_2(
        osc_z.sout, unit_vector_z, entityName + "sine_des_position_z")

    pos_des = add_vec_vec(pos_des_x, pos_des_z, entityName + "_des_pos")

    unit_vector_xd = constVector(
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], entityName + "unit_vector_xd")
    unit_vector_zd = constVector(
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], entityName + "unit_vector_zd")

    vel_des_x = mul_double_vec_2(
        osc_xd.sout, unit_vector_xd, entityName + "sine_des_velocity_x")
    vel_des_z = mul_double_vec_2(
        osc_zd.sout, unit_vector_zd, entityName + "sine_des_velocity_z")

    vel_des = add_vec_vec(vel_des_x, vel_des_z, entityName + "_des_vel")

    return pos_des, vel_des


class CircularCartesianTrajectoryGenerator(object):
    """
    generates a circular circular_trajectory_generator
    """

    def __init__(self, time_period=0.001, prefix=""):
        #
        # save arguments
        #
        self.time_period = time_period
        self.prefix = prefix

        #
        # Position
        #
        for dof in ['x', 'y', 'z']:
            # Setup the oscillation entity
            self.__dict__['osc_pos_' + dof] = dg.sot.tools.Oscillator(
                self.prefix + "osc_pos_" + dof)
            self.__dict__['osc_pos_' + dof].setTimePeriod(time_period)

        #
        # Velocity
        #
        for dof in ['x', 'y', 'z']:
            # Setup the oscillation entity
            self.__dict__['osc_vel_' + dof] = dg.sot.tools.Oscillator(
                self.prefix + "osc_vel_" + dof)
            self.__dict__['osc_vel_' + dof].setTimePeriod(time_period)
            # Fully plug the derivatives of above oscillator
            # magnitude
            self.__dict__['osc_vel_magnitude_' + dof] = Multiply_of_double(
                self.prefix + 'osc_vel_magnitude_' + dof)
            dg.plug(self.__dict__['osc_pos_' + dof].omega,
                    self.__dict__['osc_vel_magnitude_' + dof].sin0)
            dg.plug(self.__dict__['osc_pos_' + dof].magnitude,
                    self.__dict__['osc_vel_magnitude_' + dof].sin1)
            dg.plug(self.__dict__['osc_vel_magnitude_' + dof].sout,
                    self.__dict__['osc_vel_' + dof].magnitude)
            # omega
            dg.plug(self.__dict__['osc_pos_' + dof].omega,
                    self.__dict__['osc_vel_' + dof].omega)
            # phase
            self.__dict__['osc_vel_phase_' + dof] = Add_of_double(
                self.prefix + 'osc_vel_phase_' + dof)
            self.pi_by_2 = DoubleConstant(np.pi * 0.5, self.prefix + 'pi_by_2')
            dg.plug(self.pi_by_2.sout,
                    self.__dict__['osc_vel_phase_' + dof].sin1)
            dg.plug(self.__dict__['osc_pos_' + dof].phase,
                    self.__dict__['osc_vel_phase_' + dof].sin2)
            dg.plug(self.__dict__['osc_vel_phase_' + dof].sout,
                    self.__dict__['osc_vel_' + dof].phase)
            # bias
            self.__dict__['osc_vel_' + dof].bias.value = 0.0

        #
        # Create the reference cartesian position and velocity
        #
        for i, dof in enumerate(['x', 'y', 'z']):
            # create a unit vector for each dof
            unit_vector = [0.0]*6
            unit_vector[i] = 1.0
            self.__dict__['unit_vector_' + dof] = constVector(
                unit_vector, self.prefix + 'unit_vector_' + dof)
            # multiply the unit vector with the reference oscilator
            # position
            self.__dict__['pos_des_' + dof] = Multiply_double_vector(
                self.prefix + 'pos_des_' + dof)
            dg.plug(self.__dict__['osc_pos_' + dof].sout,
                    self.__dict__['pos_des_' + dof].sin1)
            dg.plug(self.__dict__['unit_vector_' + dof],
                    self.__dict__['pos_des_' + dof].sin2)
            # velocity
            self.__dict__['vel_des_' + dof] = Multiply_double_vector(
                self.prefix + 'vel_des_' + dof)
            dg.plug(self.__dict__['osc_vel_' + dof].sout,
                    self.__dict__['vel_des_' + dof].sin1)
            dg.plug(self.__dict__['unit_vector_' + dof],
                    self.__dict__['vel_des_' + dof].sin2)

        #
        # Finalize by summing up the 3 cartesian positions and velocities
        #
        # position
        self.pos_des_xy = Add_of_vector(self.prefix + 'pos_des_xy')
        dg.plug(self.pos_des_x.sout, self.pos_des_xy.sin1)
        dg.plug(self.pos_des_y.sout, self.pos_des_xy.sin2)
        self.pos_des = Add_of_vector(self.prefix + 'pos_des')
        dg.plug(self.pos_des_xy.sout, self.pos_des.sin1)
        dg.plug(self.pos_des_z.sout, self.pos_des.sin2)
        # velocity
        self.vel_des_xy = Add_of_vector(self.prefix + 'vel_des_xy')
        dg.plug(self.vel_des_x.sout, self.vel_des_xy.sin1)
        dg.plug(self.vel_des_y.sout, self.vel_des_xy.sin2)
        self.vel_des = Add_of_vector(self.prefix + 'vel_des')
        dg.plug(self.vel_des_xy.sout, self.vel_des.sin1)
        dg.plug(self.vel_des_z.sout, self.vel_des.sin2)

    def set_time_period(self, time_period):
        """
        set the time period to all oscillation entities
        """
        for i, dof in enumerate(['x', 'y', 'z']):
            self.__dict__['osc_pos_' + dof].setTimePeriod(time_period)
            self.__dict__['osc_vel_' + dof].setTimePeriod(time_period)

    def plug(self, magnitude, omega, phase, bias):
        # plug the entity
        for i, dof in enumerate(['x', 'y', 'z']):
            dg.plug(magnitude[i].sout, self.__dict__[
                    'osc_pos_' + dof].magnitude)
            dg.plug(omega[i].sout, self.__dict__['osc_pos_' + dof].omega)
            dg.plug(phase[i].sout, self.__dict__['osc_pos_' + dof].phase)
            dg.plug(bias[i].sout, self.__dict__['osc_pos_' + dof].bias)

    def trace(self, robot):
        for dof in ['x', 'y', 'z']:
            robot.add_trace(self.prefix + 'unit_vector_' + dof, 'sout')
            robot.add_trace(self.prefix + "osc_pos_" + dof, 'sout')
        robot.add_trace(self.prefix + 'pos_des', 'sout')
        robot.add_trace(self.prefix + 'vel_des', 'sout')


def cubic_interpolator(init_vector_signal, goal_vector_signal, entityName):
    """
    generate a cubic interpolation trajectory between the init_vector_signal
    and the goal_vector_signal. It returns the entity.
    """
    cubic_interpolator = dynamic_graph.sot.tools.CubicInterpolation(entityName)
    cubic_interpolator.setSamplingPeriod(0.001)
    plug(init_vector_signal, cubic_interpolator.init)
    plug(goal_vector_signal, cubic_interpolator.goal)
    cubic_interpolator.reset()
    return cubic_interpolator

#############################################################################
