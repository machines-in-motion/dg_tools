import unittest
import numpy as np
from dg_tools.dynamic_graph.dg_tools_entities  import Upsampler

class TestUpsampler(unittest.TestCase):
    def test_init(self):
        entity = Upsampler('upsampler')
        self.assertEqual(entity.name, 'upsampler')

    def test_basic(self):
        entity = Upsampler('')
        entity.init(5)

        # Setting the input value and calling the first time the output.
        # This should set the initial offset for the upsampling.
        entity.sin.value = [1, 2]
        entity.sout.recompute(2)

        # Change the value, the upsampler should keep the last value
        # sampled at t=2.
        entity.sin.value = [2, 3]
        for i in range(2, 7):
            entity.sout.recompute(i) # 2, 3, 4, 5, 6
            np.testing.assert_array_almost_equal(entity.sout.value, [1, 2])

        # At t=7, the value should get updated
        entity.sin.value = [3, 4]
        entity.sout.recompute(7)
        np.testing.assert_array_almost_equal(entity.sout.value, [3, 4])
