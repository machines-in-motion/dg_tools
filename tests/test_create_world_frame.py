import unittest
import numpy as np
import pinocchio
from dg_tools.dynamic_graph.dg_tools_entities import CreateWorldFrame


class TestSE3Offset(unittest.TestCase):
    def test_init(self):
        entity = CreateWorldFrame("se3offset")
        self.assertEqual(entity.name, "se3offset")
        self.assertTrue(entity.hasSignal("se3_offset_sin"))
        self.assertTrue(entity.hasSignal("se3_offset_sout"))

    def test_no_update_no_select(self):
        entity = CreateWorldFrame("")

        # Random input
        se3_offset_in = pinocchio.SE3.Random()
        entity.offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_offset_in)

        # checkout that the output is not changed
        entity.offset_sout.recompute(1)
        np.testing.assert_almost_equal(
            entity.se3_sout.value, np.array(6*[0]))

    def test_no_update(self):
        entity = CreateWorldFrame("")
        entity.set_which_dofs(np.array(6 * [1]))

        # Random input
        se3_offset_in = pinocchio.SE3.Random()
        entity.offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_offset_in)

        # checkout that the output is not changed
        entity.offset_sout.recompute(1)
        np.testing.assert_almost_equal(
            entity.se3_sout.value, np.array(6*[0]))

    def test_use_all(self):
        entity = CreateWorldFrame("")
        entity.update_offset()
        entity.set_which_dofs(np.array(6 * [1]))

        # Random input
        se3_offset_in = pinocchio.SE3.Random()
        entity.offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_offset_in)

        # checkout that the output is not changed
        entity.se3_sout.recompute(1)
        np.testing.assert_almost_equal(entity.se3_sout.value, identity)

    # def test_full_offset_xyz_null(self):

    #     entity = CreateWorldFrame("")
    #     entity.update_offset()
    #     entity.set_which_dofs(np.array(6 * [1]))

    #     # Identity
    #     identity = pinocchio.SE3ToXYZQUAT(pinocchio.SE3.Identity())

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()
    #     entity.offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)

    #     # Offset == Input
    #     se3_offset = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset[3] = 0
    #     se3_offset[4] = 0
    #     se3_offset[5] = 0
    #     se3_offset[6] = 1
    #     entity.se3_offset_sin.value = se3_offset

    #     # checkout that the output is not changed
    #     entity.se3_sout.recompute(1)
    #     np.testing.assert_almost_equal(entity.se3_sout.value[:3], np.array(3 * [0]))
    #     np.testing.assert_almost_equal(
    #         entity.se3_sout.value[3:], entity.offset_sin.value[3:]
    #     )

    # def test_full_offset_x_null(self):

    #     entity = CreateWorldFrame("")
    #     entity.update_offset()
    #     entity.set_which_dofs(np.array(6 * [1]))

    #     # Identity
    #     identity = pinocchio.SE3ToXYZQUAT(pinocchio.SE3.Identity())

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()
    #     entity.offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)

    #     # Offset == Input
    #     se3_offset = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset[1] = 0
    #     se3_offset[2] = 0
    #     se3_offset[3] = 0
    #     se3_offset[4] = 0
    #     se3_offset[5] = 0
    #     se3_offset[6] = 1
    #     entity.se3_offset_sin.value = se3_offset

    #     # checkout that the output is not changed
    #     entity.se3_sout.recompute(1)
    #     np.testing.assert_almost_equal(
    #         entity.se3_sout.value[:3],
    #         np.array([0] + entity.offset_sin.value[1:3].tolist()),
    #     )
    #     np.testing.assert_almost_equal(
    #         entity.se3_sout.value[3:], entity.offset_sin.value[3:]
    #     )

    # def test_full_offset_y_null(self):

    #     entity = CreateWorldFrame("")
    #     entity.update_offset()
    #     entity.set_which_dofs(np.array(6 * [1]))

    #     # Identity
    #     identity = pinocchio.SE3ToXYZQUAT(pinocchio.SE3.Identity())

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()
    #     entity.offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)

    #     # Offset == Input
    #     se3_offset = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset[0] = 0
    #     se3_offset[2] = 0
    #     se3_offset[3] = 0
    #     se3_offset[4] = 0
    #     se3_offset[5] = 0
    #     se3_offset[6] = 1
    #     entity.se3_offset_sin.value = se3_offset

    #     # checkout that the output is not changed
    #     entity.se3_sout.recompute(1)
    #     np.testing.assert_almost_equal(
    #         entity.se3_sout.value[:3],
    #         np.array([entity.offset_sin.value[0]] + [0] + [entity.offset_sin.value[2]]),
    #     )
    #     np.testing.assert_almost_equal(
    #         entity.se3_sout.value[3:], entity.offset_sin.value[3:]
    #     )

    # def test_full_offset_z_null(self):

    #     entity = CreateWorldFrame("")
    #     entity.update_offset()
    #     entity.set_which_dofs(np.array(6 * [1]))

    #     # Identity
    #     identity = pinocchio.SE3ToXYZQUAT(pinocchio.SE3.Identity())

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()
    #     entity.offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)

    #     # Offset == Input
    #     se3_offset = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset[0] = 0
    #     se3_offset[1] = 0
    #     se3_offset[3] = 0
    #     se3_offset[4] = 0
    #     se3_offset[5] = 0
    #     se3_offset[6] = 1
    #     entity.se3_offset_sin.value = se3_offset

    #     # checkout that the output is not changed
    #     entity.se3_sout.recompute(1)
    #     np.testing.assert_almost_equal(
    #         entity.se3_sout.value[:3],
    #         np.array(entity.offset_sin.value[:2].tolist() + [0]),
    #     )
    #     np.testing.assert_almost_equal(
    #         entity.se3_sout.value[3:], entity.offset_sin.value[3:]
    #     )

    # def test_full_offset_roll_null(self):

    #     entity = CreateWorldFrame("")
    #     entity.update_offset()
    #     entity.set_which_dofs(np.array(6 * [1]))

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()
    #     rpy_input = pinocchio.rpy.matrixToRpy(se3_input.rotation)
    #     rpy_input[1] = 0
    #     rpy_input[2] = 0
    #     se3_input.rotation = pinocchio.rpy.rpyToMatrix(rpy_input)
    #     entity.offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)

    #     # Offset == Input
    #     se3_offset = pinocchio.SE3.Identity()
    #     se3_offset.rotation = pinocchio.rpy.rpyToMatrix(
    #         np.array(
    #             [
    #                 rpy_input[0],
    #                 0,
    #                 0,
    #             ]
    #         )
    #     )
    #     entity.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_offset)

    #     # checkout that the output is not changed
    #     entity.se3_sout.recompute(1)
    #     se3_out = pinocchio.XYZQUATToSE3(entity.se3_sout.value)
    #     rpy_out = pinocchio.rpy.matrixToRpy(se3_out.rotation)
    #     np.testing.assert_almost_equal(rpy_out, np.array([0, 0, 0]))

    # def test_full_offset_pitch_null(self):

    #     entity = CreateWorldFrame("")
    #     entity.update_offset()
    #     entity.set_which_dofs(np.array(6 * [1]))

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()
    #     rpy_input = pinocchio.rpy.matrixToRpy(se3_input.rotation)
    #     rpy_input[0] = 0
    #     rpy_input[2] = 0
    #     se3_input.rotation = pinocchio.rpy.rpyToMatrix(rpy_input)
    #     entity.offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)

    #     # Offset == Input
    #     se3_offset = pinocchio.SE3.Identity()
    #     se3_offset.rotation = pinocchio.rpy.rpyToMatrix(
    #         np.array(
    #             [
    #                 0,
    #                 rpy_input[1],
    #                 0,
    #             ]
    #         )
    #     )
    #     entity.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_offset)

    #     # checkout that the output is not changed
    #     entity.se3_sout.recompute(1)
    #     se3_out = pinocchio.XYZQUATToSE3(entity.se3_sout.value)
    #     rpy_out = pinocchio.rpy.matrixToRpy(se3_out.rotation)
    #     np.testing.assert_almost_equal(rpy_out, np.array([0, 0, 0]))

    # def test_full_offset_yaw_null(self):

    #     entity = CreateWorldFrame("")
    #     entity.update_offset()
    #     entity.set_which_dofs(np.array(6 * [1]))

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()
    #     rpy_input = pinocchio.rpy.matrixToRpy(se3_input.rotation)
    #     rpy_input[0] = 0
    #     rpy_input[1] = 0
    #     se3_input.rotation = pinocchio.rpy.rpyToMatrix(rpy_input)
    #     entity.offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)

    #     # Offset == Input
    #     se3_offset = pinocchio.SE3.Identity()
    #     se3_offset.rotation = pinocchio.rpy.rpyToMatrix(
    #         np.array(
    #             [
    #                 0,
    #                 0,
    #                 rpy_input[2],
    #             ]
    #         )
    #     )
    #     entity.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_offset)

    #     # checkout that the output is not changed
    #     entity.se3_sout.recompute(1)
    #     se3_out = pinocchio.XYZQUATToSE3(entity.se3_sout.value)
    #     rpy_out = pinocchio.rpy.matrixToRpy(se3_out.rotation)
    #     np.testing.assert_almost_equal(rpy_out, np.array([0, 0, 0]))

    # def test_full_vs_partial_offset_x(self):

    #     entity_full = CreateWorldFrame("")
    #     entity_full.update_offset()
    #     entity_full.set_which_dofs(np.array(6 * [1]))

    #     entity_part = CreateWorldFrame("")
    #     entity_part.update_offset()

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()

    #     entity_full.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset[1] = 0
    #     se3_offset[2] = 0
    #     se3_offset[3] = 0
    #     se3_offset[4] = 0
    #     se3_offset[5] = 0
    #     se3_offset[6] = 1
    #     entity_full.se3_offset_sin.value = se3_offset
    #     entity_full.se3_sout.recompute(1)

    #     entity_part.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.set_which_dofs(np.array([1, 0, 0, 0, 0, 0]))
    #     entity_part.se3_sout.recompute(1)

    #     np.testing.assert_almost_equal(
    #         entity_full.se3_sout.value, entity_part.se3_sout.value
    #     )

    # def test_full_vs_partial_offset_y(self):

    #     entity_full = CreateWorldFrame("")
    #     entity_full.update_offset()
    #     entity_full.set_which_dofs(np.array(6 * [1]))

    #     entity_part = CreateWorldFrame("")
    #     entity_part.update_offset()

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()

    #     entity_full.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset[0] = 0
    #     se3_offset[2] = 0
    #     se3_offset[3] = 0
    #     se3_offset[4] = 0
    #     se3_offset[5] = 0
    #     se3_offset[6] = 1
    #     entity_full.se3_offset_sin.value = se3_offset
    #     entity_full.se3_sout.recompute(2)

    #     entity_part.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.set_which_dofs(np.array([0, 1, 0, 0, 0, 0]))
    #     entity_part.se3_sout.recompute(2)
    #     np.testing.assert_almost_equal(
    #         entity_full.se3_sout.value, entity_part.se3_sout.value
    #     )
    
    # def test_full_vs_partial_offset_z(self):

    #     entity_full = CreateWorldFrame("")
    #     entity_full.update_offset()
    #     entity_full.set_which_dofs(np.array(6 * [1]))

    #     entity_part = CreateWorldFrame("")
    #     entity_part.update_offset()

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()

    #     entity_full.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset[0] = 0
    #     se3_offset[1] = 0
    #     se3_offset[3] = 0
    #     se3_offset[4] = 0
    #     se3_offset[5] = 0
    #     se3_offset[6] = 1
    #     entity_full.se3_offset_sin.value = se3_offset
    #     entity_full.se3_sout.recompute(2)

    #     entity_part.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.set_which_dofs(np.array([0, 0, 1, 0, 0, 0]))
    #     entity_part.se3_sout.recompute(2)
    #     np.testing.assert_almost_equal(
    #         entity_full.se3_sout.value, entity_part.se3_sout.value
    #     )

    # def test_full_vs_partial_offset_roll(self):

    #     entity_full = CreateWorldFrame("")
    #     entity_full.update_offset()
    #     entity_full.set_which_dofs(np.array(6 * [1]))

    #     entity_part = CreateWorldFrame("")
    #     entity_part.update_offset()

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()
    #     rpy_input = pinocchio.rpy.matrixToRpy(se3_input.rotation)
    #     rpy_input[1] = 0
    #     rpy_input[2] = 0
    #     se3_input.rotation = pinocchio.rpy.rpyToMatrix(rpy_input)

    #     entity_full.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset = pinocchio.SE3.Identity()
    #     se3_offset.rotation = pinocchio.rpy.rpyToMatrix(
    #         np.array(
    #             [
    #                 rpy_input[0],
    #                 0,
    #                 0,
    #             ]
    #         )
    #     )
    #     entity_full.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_offset)
    #     entity_full.se3_sout.recompute(2)

    #     entity_part.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.set_which_dofs(np.array([0, 0, 0, 1, 0, 0]))
    #     entity_part.se3_sout.recompute(2)
    #     np.testing.assert_almost_equal(
    #         entity_full.se3_sout.value, entity_part.se3_sout.value
    #     )

    # def test_full_vs_partial_offset_pitch(self):

    #     entity_full = CreateWorldFrame("")
    #     entity_full.update_offset()
    #     entity_full.set_which_dofs(np.array(6 * [1]))

    #     entity_part = CreateWorldFrame("")
    #     entity_part.update_offset()

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()
    #     rpy_input = pinocchio.rpy.matrixToRpy(se3_input.rotation)
    #     rpy_input[0] = 0
    #     rpy_input[2] = 0
    #     se3_input.rotation = pinocchio.rpy.rpyToMatrix(rpy_input)

    #     entity_full.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset = pinocchio.SE3.Identity()
    #     se3_offset.rotation = pinocchio.rpy.rpyToMatrix(
    #         np.array(
    #             [
    #                 0,
    #                 rpy_input[1],
    #                 0,
    #             ]
    #         )
    #     )
    #     entity_full.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_offset)
    #     entity_full.se3_sout.recompute(2)

    #     entity_part.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.set_which_dofs(np.array([0, 0, 0, 0, 1, 0]))
    #     entity_part.se3_sout.recompute(2)
    #     np.testing.assert_almost_equal(
    #         entity_full.se3_sout.value, entity_part.se3_sout.value
    #     )


    # def test_full_vs_partial_offset_yaw(self):

    #     entity_full = CreateWorldFrame("")
    #     entity_full.update_offset()
    #     entity_full.set_which_dofs(np.array(6 * [1]))

    #     entity_part = CreateWorldFrame("")
    #     entity_part.update_offset()

    #     # Random input
    #     se3_input = pinocchio.SE3.Random()
    #     rpy_input = pinocchio.rpy.matrixToRpy(se3_input.rotation)
    #     rpy_input[0] = 0
    #     rpy_input[1] = 0
    #     se3_input.rotation = pinocchio.rpy.rpyToMatrix(rpy_input)

    #     entity_full.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     se3_offset = pinocchio.SE3.Identity()
    #     se3_offset.rotation = pinocchio.rpy.rpyToMatrix(
    #         np.array(
    #             [
    #                 0,
    #                 0,
    #                 rpy_input[2],
    #             ]
    #         )
    #     )
    #     entity_full.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_offset)
    #     entity_full.se3_sout.recompute(2)

    #     entity_part.se3_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.se3_offset_sin.value = pinocchio.SE3ToXYZQUAT(se3_input)
    #     entity_part.set_which_dofs(np.array([0, 0, 0, 0, 0, 1]))
    #     entity_part.se3_sout.recompute(2)
    #     np.testing.assert_almost_equal(
    #         entity_full.se3_sout.value, entity_part.se3_sout.value
    #     )