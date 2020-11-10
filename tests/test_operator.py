import unittest
import numpy as np

from dg_tools.dynamic_graph.dg_tools_entities  import PoseQuaternionToPoseRPY

import pinocchio as pin

def posquat2posrpy(pos):
    pos = np.matrix(pos).T
    se3 = pin.utils.XYZQUATToSe3(pos)
    return np.array(np.hstack([pos.T[:, :3], pin.rpy.matrixToRpy(se3.rotation).T])).reshape(-1)

class TestPoseQuaternionToPoseRPY(unittest.TestCase):
    def test_basic(self):
        op = PoseQuaternionToPoseRPY('')
        op.sin.value = [0, 1, 2, 0, 0, 0, 1]

        op.sout.recompute(10)
        np.testing.assert_array_equal(op.sout.value, [0, 1, 2, 0, 0, 0])

    def test_rotated(self):
        q0 = (0.34245364134360234, 0.3289527217213123, 0.1978762653232481,
            0.002508853212199517, 0.016383906408659818, 0.20479396648554946, 0.9786647559596021)

        pq2pr = PoseQuaternionToPoseRPY('')
        pq2pr.sin.value = q0
        pq2pr.sout.recompute(1)

        np.testing.assert_allclose(posquat2posrpy(q0), pq2pr.sout.value, rtol=1e-4)
