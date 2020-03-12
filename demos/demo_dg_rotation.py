#!/usr/bin/env python

# @namespace dg_tools.demos
""" Yaw decomposition demos

    @file
    @copyright Copyright (c) 2017-2019,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

# Python 3 compatibility, has to be called just after the hashbang.
from __future__ import print_function, division

import numpy as np
import pinocchio
from pinocchio import Quaternion, SE3, se3ToXYZQUAT, XYZQUATToSe3
from pinocchio.rpy import matrixToRpy, rpyToMatrix
import time
import os

from robot_properties_solo.config import Solo12Config

from dynamic_graph import plug
from dynamic_graph.sot.core.operator import PoseRollPitchYawToMatrixHomo
from dynamic_graph_manager.dg_tools import PoseQuaternionToPoseRPY
from dg_tools.utils import VectorSignal

# Load the robot urdf.
robot = Solo12Config.buildRobotWrapper()

# Setup the display (connection to gepetto viewer) and load the robot model.
robot.initDisplay(loadModel=True)

# Create a first initial position for the robot. Both legs are bent inwards.
q = np.matrix(Solo12Config.initial_configuration).T

# Display the configuration in the viewer.
robot.display(q)

# Example of moving the robot forward and updating the display every time.
N = 1000
for i in range(N):
    yaw = 10.0 * float(i) / float(N)

    q[0:3].fill(0.0)

    base_se3 = SE3(pinocchio.rpy.rpyToMatrix(np.matrix([[0.1], [0.2], [yaw]])),
                   q[0:3])
    q[0:7] = se3ToXYZQUAT(base_se3).reshape(7, 1)

    # Build graph: zero roll and pitch
    xyzquat_to_xyzrpy = PoseQuaternionToPoseRPY("xyzquat_to_xyzrpy")
    xyzquat_to_xyzrpy.sin.value = [0, 0, 0, 0, 0, 0, 1]
    xyzrpy_base_yaw = VectorSignal([0, 0, 0, 0, 0]).concat(
        VectorSignal(xyzquat_to_xyzrpy.sout, 6)[5:6])
    base_yaw_homo = PoseRollPitchYawToMatrixHomo("base_yaw_homo")
    plug(xyzrpy_base_yaw, base_yaw_homo.sin)

    # feed the graph, recompute and get the result
    xyzquat_to_xyzrpy.sin.value = q[0:7].reshape(7).tolist()[0]
    print(xyzquat_to_xyzrpy.sin.value)
    base_yaw_homo.sout.recompute(i)
    base_homo = np.asmatrix(base_yaw_homo.sout.value)
    new_base_se3 = SE3(base_homo[:3, :3], base_homo[:3, 3]) 
    q[0:7] = se3ToXYZQUAT(new_base_se3).reshape(7, 1)

    # Display the robot
    robot.display(q)
    time.sleep(0.001)
