import unittest
import numpy as np

from dynamic_graph_manager.dg_tools import VectorIntegrator

class TestVectorIntegrator(unittest.TestCase):
    def test_basic(self):
        op = VectorIntegrator('vec_integrator')
        op.sin.value = [1, 2, 3]

        op.sout.recompute(3)
        np.testing.assert_array_almost_equal(op.sout.value, (0.001, 0.002, 0.003))
        op.sout.recompute(4)
        np.testing.assert_array_almost_equal(op.sout.value, (0.002, 0.004, 0.006))

        op.sin.value = [2, 3, 5]
        op.sout.recompute(5)
        np.testing.assert_array_almost_equal(op.sout.value, (0.004, 0.007, 0.011))
