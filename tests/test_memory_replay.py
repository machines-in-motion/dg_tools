import unittest
import numpy as np

from dynamic_graph_manager.dg_tools import MemoryReplay

class TestMemoryReplay(unittest.TestCase):
    def test_basic(self):
        a = np.random.rand(2, 3)
        a = tuple([tuple(b) for b in a])
        entity = MemoryReplay('')

        # Init from the matrix.
        entity.init(a)

        # Replay the memory.
        entity.sout.recompute(10)
        np.testing.assert_array_equal(entity.sout.value, a[0])

        entity.sout.recompute(11)
        np.testing.assert_array_equal(entity.sout.value, a[1])

        # The entity should keep repeating the last value.
        entity.sout.recompute(12)
        np.testing.assert_array_equal(entity.sout.value, a[1])

        # Check if rewinding
        entity.rewind()
        entity.sout.recompute(13)
        np.testing.assert_array_equal(entity.sout.value, a[0])
