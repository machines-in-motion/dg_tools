import unittest
import numpy as np

from dg_tools.dynamic_graph.dg_tools_entities  import HistoryRecorder

class TestHistoryRecorder(unittest.TestCase):
    def test_basic(self):
        entity = HistoryRecorder('')
        entity.init(3)

        # Setting the input value and calling the first time the output.
        # This should set the initial offset for the upsampling.
        entity.sin.value = [1, 2]
        entity.sout.recompute(2)
        np.testing.assert_array_equal(entity.sout.value, [1, 2, 1, 2, 1, 2])

        entity.sin.value = [3, 4]
        entity.sout.recompute(3)
        np.testing.assert_array_equal(entity.sout.value, [1, 2, 1, 2, 3, 4])

        entity.sin.value = [5, 6]
        entity.sout.recompute(5)
        np.testing.assert_array_equal(entity.sout.value, [1, 2, 3, 4, 5, 6])

        entity.sin.value = [7, 8]
        entity.sout.recompute(6)
        np.testing.assert_array_equal(entity.sout.value, [3, 4, 5, 6, 7, 8])
