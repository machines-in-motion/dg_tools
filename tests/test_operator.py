import unittest
import numpy as np
import pinocchio

from dynamic_graph import plug
from dg_tools.dynamic_graph.dg_tools_entities import (
    PoseQuaternionToPoseRPY,
    PoseRPYToPoseQuaternion,
)


class TestPoseQuaternionToPoseRPY(unittest.TestCase):
    def test_signal_assignement(self):
        """ Test the signal assignement. """
        op = PoseQuaternionToPoseRPY("")
        op.sin.value = np.array([0, 1, 2, 0, 0, 0, 1])

        op.sout.recompute(10)
        np.testing.assert_array_equal(op.sout.value, np.array([0, 1, 2, 0, 0, 0]))

    def test_against_pinocchio(self):
        """ Test against pinocchio conversion tools """

        # Setup

        # Create a Random SE3 object.
        pin_se3_pose_rot_mat = pinocchio.SE3.Random()

        # Convert the rotation in roll-pitch-yaw.
        pin_rpy = pinocchio.rpy.matrixToRpy(pin_se3_pose_rot_mat.rotation).T
        pin_se3_pose_rpy = np.array(
            np.hstack([pin_se3_pose_rot_mat.translation, pin_rpy])
        )

        # Convert the rotation in quaternion.
        pin_se3_pose_quat = pinocchio.SE3ToXYZQUAT(pin_se3_pose_rot_mat)

        # Create the entity and convert the se3(pose, quat) -> se3(pose, rpy).
        pq2pr = PoseQuaternionToPoseRPY("")
        pq2pr.sin.value = np.array(pin_se3_pose_quat)
        pq2pr.sout.recompute(1)
        entity_se3_pose_rpy = pq2pr.sout.value

        # Testing
        np.testing.assert_allclose(pin_se3_pose_rpy, entity_se3_pose_rpy, rtol=1e-6)

    def test_double_convertion(self):
        # Setup

        # Create a Random SE3 object.
        pin_se3_pose_rot_mat = pinocchio.SE3.Random()
        # Convert the rotation in quaternion.
        init_se3_pose_quat = np.array(pinocchio.SE3ToXYZQUAT(pin_se3_pose_rot_mat))

        # Create the small graph.
        quat2rpy = PoseQuaternionToPoseRPY("")
        rpy2quat = PoseRPYToPoseQuaternion("")
        plug(quat2rpy.sout, rpy2quat.sin)
        
        # Feed the graph.
        quat2rpy.sin.value = init_se3_pose_quat
        # Evaluate the graph.
        rpy2quat.sout.recompute(1)
        # Extract the result.
        final_se3_pose_quat = rpy2quat.sout.value

        # Make the quaternions w positive for testing:
        if init_se3_pose_quat[-1] < 0.0:
            init_se3_pose_quat[-4:] *= -1
        if final_se3_pose_quat[-1] < 0.0:
            final_se3_pose_quat[-4:] *= -1

        # Testing
        np.testing.assert_allclose(init_se3_pose_quat, final_se3_pose_quat, rtol=1e-4)
