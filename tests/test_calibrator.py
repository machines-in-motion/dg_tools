import unittest
import numpy as np
from dynamic_graph_manager.dg_tools import Calibrator

class TestCalibrator(unittest.TestCase):
    def test_init(self):
        calib = Calibrator('calibrator')
        self.assertEqual(calib.name, 'calibrator')

    def test_basic(self):
        calib = Calibrator('')
        
        calib.raw_position.value =(0.0,0.0)
        calib.velocity.value = (0.0,0.0)
        calib.calibration_torque.value = (1.0,-1.0)
        calib.hardstop2zero.value = (0.5, 0.5)

        calib.calibrated_position.recompute(0)
        # at first, entity should be uncalibrated, and output is input+hardstop
        np.testing.assert_array_equal(calib.calibrated_position.value,
            np.add(np.array(calib.raw_position.value),
            np.array(calib.hardstop2zero.value)))       
        # TODO: test flag out
        # TODO: test after calibration is finished, both flag out and position
        # TODO: test calibration torque before and after calibration is finished

