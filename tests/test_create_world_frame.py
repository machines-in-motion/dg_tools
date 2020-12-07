import unittest
import numpy as np
import pinocchio
from dg_tools.dynamic_graph.dg_tools_entities import CreateWorldFrame


class TestCreateWorldFrame(unittest.TestCase):
    def test_init(self):
        entity = CreateWorldFrame("se3offset")
        self.assertEqual(entity.name, "se3offset")
        self.assertTrue(entity.hasSignal("frame_sin"))
        self.assertTrue(entity.hasSignal("world_frame_sout"))
        self.assertTrue(callable(getattr(entity, "update", None)))
        self.assertTrue(callable(getattr(entity, "set_which_dofs", None)))

    def test_no_update_no_select(self):
        entity = CreateWorldFrame("")

        # Random input
        identity_frame = pinocchio.SE3.Identity()
        random_frame = pinocchio.SE3.Random()
        entity.frame_sin.value = pinocchio.SE3ToXYZQUAT(random_frame)

        # checkout that the output is not changed
        entity.world_frame_sout.recompute(1)
        np.testing.assert_almost_equal(
            entity.world_frame_sout.value, pinocchio.SE3ToXYZQUAT(identity_frame)
        )

    def test_no_update_use_all(self):
        entity = CreateWorldFrame("")
        entity.set_which_dofs(np.array(6 * [1]))

        # Random input
        identity_frame = pinocchio.SE3.Identity()
        random_frame = pinocchio.SE3.Random()
        entity.frame_sin.value = pinocchio.SE3ToXYZQUAT(random_frame)

        # checkout that the output is not changed
        entity.world_frame_sout.recompute(1)
        np.testing.assert_almost_equal(
            entity.world_frame_sout.value, pinocchio.SE3ToXYZQUAT(identity_frame)
        )

    def test_use_all(self):
        entity = CreateWorldFrame("")
        entity.update()
        entity.set_which_dofs(np.array(6 * [1]))

        # Random input
        random_frame = pinocchio.SE3.Random()
        entity.frame_sin.value = pinocchio.SE3ToXYZQUAT(random_frame)

        # checkout that the output is not changed
        entity.world_frame_sout.recompute(1)
        np.testing.assert_almost_equal(
            entity.world_frame_sout.value, entity.frame_sin.value
        )

    def test_full_offset_xyz_null(self):

        entity = CreateWorldFrame("")
        entity.update()
        entity.set_which_dofs(np.array([0, 0, 0, 1, 1, 1]))

        # Random input
        random_frame = pinocchio.SE3.Random()
        entity.frame_sin.value = pinocchio.SE3ToXYZQUAT(random_frame)

        # checkout that the output is not changed
        entity.world_frame_sout.recompute(1)
        np.testing.assert_almost_equal(
            entity.world_frame_sout.value, 3 * [0] + entity.frame_sin.value[3:].tolist()
        )
