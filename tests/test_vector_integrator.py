import unittest
import numpy as np

from dg_tools.dynamic_graph.dg_tools_entities import VectorIntegrator


class TestVectorIntegrator(unittest.TestCase):
    def test_basic(self):
        op = VectorIntegrator("vec_integrator")
        op.sin.value = np.array([1, 2, 3])

        op.sout.recompute(3)
        np.testing.assert_array_almost_equal(
            op.sout.value, np.array([0.001, 0.002, 0.003])
        )
        op.sout.recompute(4)
        np.testing.assert_array_almost_equal(
            op.sout.value, np.array([0.002, 0.004, 0.006])
        )

        op.sin.value = np.array([2, 3, 5])
        op.sout.recompute(5)
        np.testing.assert_array_almost_equal(
            op.sout.value, np.array([0.004, 0.007, 0.011])
        )
