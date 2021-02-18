#!/usr/bin/env python

""" Trajectory generator demos.

    Copyright (c) 2017-2019,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

import numpy as np
import matplotlib.pyplot as plt
from dg_tools.math_small_entities import ConstantDouble
from dg_tools.traj_generators import CircularCartesianTrajectoryGenerator

if __name__ == "__main__":

    traj_gen = CircularCartesianTrajectoryGenerator("traj_gen")

    # parameters:
    duration = 20.0
    time_period = 0.001

    magnitude_x = ConstantDouble(0.1, "magnitude_x")
    magnitude_y = ConstantDouble(0.2, "magnitude_y")
    magnitude_z = ConstantDouble(0.3, "magnitude_z")
    magnitude = [magnitude_x, magnitude_y, magnitude_z]

    omega_x = ConstantDouble(0.1, "omega_x")
    omega_y = ConstantDouble(0.2, "omega_y")
    omega_z = ConstantDouble(0.3, "omega_z")
    omega = [omega_x, omega_y, omega_z]

    phase_x = ConstantDouble(0.1, "phase_x")
    phase_y = ConstantDouble(0.2, "phase_y")
    phase_z = ConstantDouble(0.3, "phase_z")
    phase = [phase_x, phase_y, phase_z]

    bias_x = ConstantDouble(0.1, "bias_x")
    bias_y = ConstantDouble(0.2, "bias_y")
    bias_z = ConstantDouble(0.3, "bias_z")
    bias = [bias_x, bias_y, bias_z]

    traj_gen.set_time_period(time_period)
    traj_gen.plug(magnitude, omega, phase, bias)

    nsamples = int(duration / time_period)
    t = np.linspace(0, duration, nsamples, endpoint=False)

    des_pos = []
    des_vel = []
    for i in range(t.size):
        traj_gen.des_pos.recompute(i)
        des_pos += [traj_gen.des_pos.value]
        des_vel += [traj_gen.des_vel.value]

    des_pos = np.array(des_pos)
    des_vel = np.array(des_vel)
    plt.plot(t, des_pos[:, 0], label="x")
    plt.plot(t, des_pos[:, 1], label="y")
    plt.plot(t, des_pos[:, 2], label="z")
    plt.xlabel("time (seconds)")
    plt.grid(True)
    plt.axis("tight")
    plt.legend(loc="upper left")

    plt.show()
