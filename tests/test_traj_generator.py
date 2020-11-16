import unittest
from copy import deepcopy

from dg_tools.traj_generators  import CircularCartesianTrajectoryGenerator
from dg_tools.math_small_entities import ConstantDouble

class TestPreviousValue(unittest.TestCase):
    def test_basic(self):
        # Create the entity
        entity = CircularCartesianTrajectoryGenerator()
        
        # Set the parameters
        magnitude = ConstantDouble(5.0 , "magnitude")
        omega = ConstantDouble(3.1415 , "omega")
        phase = ConstantDouble(0.5 , "phase")
        bias = ConstantDouble(2.0 , "bias")

        # Plug the input parameters
        entity.plug(3*[magnitude], 3*[omega], 3*[phase], 3*[bias])

        # Collect some data
        dt = 0.001
        time = []
        x = []
        y = []
        z = []
        for i in range(5000):
            time.append(i * dt)
            entity.des_pos_xy.sout.recompute(i)
            x.append(entity.des_pos_xy.sout.value[0])
            y.append(entity.des_pos_xy.sout.value[1])
            z.append(entity.des_pos_xy.sout.value[2])

        self.assertEqual(x, y)
        self.assertEqual(y, z)
        self.assertEqual(x, z)