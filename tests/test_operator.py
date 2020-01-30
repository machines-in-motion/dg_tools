import unittest
import numpy as np

from dynamic_graph_manager.dg_tools import PoseQuaternionToPoseRPY

class TestPoseQuaternionToPoseRPY(unittest.TestCase):
    def test_basic(self):
        op = PoseQuaternionToPoseRPY('')
        op.sin.value = [0, 1, 2, 0, 0, 0, 1]

        op.sout.recompute(10)
        np.testing.assert_array_equal(op.sout.value, [0, 1, 2, 0, 0, 0])
