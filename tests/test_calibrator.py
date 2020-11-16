import unittest
import numpy as np
from dg_tools.dynamic_graph.dg_tools_entities import Calibrator


class TestCalibrator(unittest.TestCase):
    def test_init(self):
        calib = Calibrator("calibrator")
        self.assertEqual(calib.name, "calibrator")

    def test_basic(self):
        calib = Calibrator("")

        calib.raw_position.value = np.array([0.0, 0.0])
        calib.velocity.value = np.array([0.0, 0.0])
        calib.calibration_torque.value = np.array([1.0, -1.0])
        calib.hardstop2zero.value = np.array([ 0.5, 0.5])

        calib.calibrated_position.recompute(0)
        # at first, entity should be uncalibrated, and output is input+hardstop
        np.testing.assert_array_equal(
            calib.calibrated_position.value,
            np.add(
                np.array(calib.raw_position.value), np.array(calib.hardstop2zero.value)
            ),
        )
        # TODO: test flag out
        # TODO: test after calibration is finished, both flag out and position
        # TODO: test calibration torque before and after calibration is finished
