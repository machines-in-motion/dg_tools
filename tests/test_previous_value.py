import unittest
import numpy as np

from dg_tools.dynamic_graph.dg_tools_entities  import PreviousValue

class TestPreviousValue(unittest.TestCase):
    def test_basic(self):
        entity = PreviousValue('')
        entity.init(3)

        entity.sprev.recompute(0)
        np.testing.assert_array_equal(entity.sprev.value, [0, 0, 0])

        old_value = [0, 1, 2]
        entity.sin.value = old_value
        entity.sout.recompute(1)
        np.testing.assert_array_equal(entity.sout.value, old_value)

        entity.sprev.recompute(1)
        np.testing.assert_array_equal(entity.sprev.value, old_value)

        entity.sprev.recompute(2)
        np.testing.assert_array_equal(entity.sprev.value, old_value)
