"""
@package py_dg_tools
@author Avadesh Meduri
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-03-01
@brief Contains generic impedance control functions that will be used by the
impedance_controller
"""

#
# Imports
#
import numpy as np
from dynamic_graph import plug

from dynamic_graph.sot.core.double_constant import DoubleConstant
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.matrix_constant import MatrixConstant
from dynamic_graph.sot.core.operator import (
    Add_of_double,
    Add_of_vector,
    Multiply_double_vector,
    Substract_of_double,
    Substract_of_vector,
    Multiply_matrix_vector,
    MatrixTranspose,
    MatrixHomoToPose,
    Component_of_vector,
    Selec_of_vector,
    Stack_of_vector,
    Multiply_of_double,
)
# from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
# from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double

def stack_zero(vec, entityName=''):
    """
    ## This function stacks a zeros before the vector
    ## Input : Constant vector (not numpy arrays)
          : Constant vector (not numpy arrays)
          : size of first vector (int)
          : size of first vector (int)
    """
    zero = VectorConstant("zero")
    zero.sout.value = (0.,)

    op = Stack_of_vector(entityName)
    op.selec1(0, 1)
    op.selec2(0, 2)
    plug(zero.sout, op.sin1)
    plug(vec, op.sin2)
    return op.sout


def add_doub_doub_2(db1, db2, entityName):
    add = Add_of_double(entityName)
    plug(db1, add.signal('sin1'))
    plug(db2, add.signal('sin2'))
    return add.sout


def mul_double_vec_2(doub, vec, entityName):
    # need double as a signal
    mul = Multiply_double_vector(entityName)
    if isinstance(doub, float):
        mul.sin1.value = doub
    else:
        plug(doub, mul.signal('sin1'))
    plug(vec, mul.signal('sin2'))
    return mul.sout


def scale_values(double, scale, entityName):
    mul = Multiply_of_double(entityName)
    mul.sin0.value = scale
    plug(double, mul.sin1)
    return mul.sout


def mul_doub_doub(db1, db2, entityName):
    dif1 = Multiply_of_double(entityName)
    if isinstance(db1, float):
        dif1.sin0.value = db1
    else:
        plug(db1, dif1.sin0)
    plug(db2, dif1.sin1)
    return dif1.sout


#
# Initialisers
#
class ConstantVector(object):
    """
    Define a nice interface to create constant vector
    """

    def __init__(self, vec, entity_name):
        self.vec = VectorConstant(entity_name)
        self.set_vec(vec)
        self.sout = self.vec.sout

    def set_vec(self, vec):
        if isinstance(vec, list):
            self.vec.sout.value = np.array(vec)
        else:
            self.vec.sout.value = vec

    def get_vec(self):
        return self.vec.sout.value


def constVector(val, entityName=''):
    """
    ## This function initialises an constant vector
    ## Input : array (python list)
    """
    op = VectorConstant(entityName).sout
    op.value = list(val)
    return op


def matrixConstant(val):
    """
    ## This function initialises an constant matrix
    ## Input : matrix (python array)
    """
    op = MatrixConstant("").sout
    op.value = val
    return op


class ConstantDouble(object):
    """
    Wrapper around the double constant entity.
    """

    def __init__(self, value, entity_name=""):
        # create a "double + double"
        self.double = DoubleConstant(entity_name)
        # initialize both value to 0.0
        self.set_value(value)
        self.sout = self.double.sout

    def set_value(self, value):
        self.double.set(value)

    def get_value(self):
        return self.double.sout.value


"""
Operators
"""


class Stack2Vectors(object):
    """
    Define an easier interface to simple stack 2 vectors
    """

    def __init__(self, sin1, sin2, size1, size2, entity_name=""):
        self.op = Stack_of_vector(entity_name)
        self.op.selec1(0, size1)
        self.op.selec2(0, size2)
        self.sin1 = self.op.sin1
        self.sin2 = self.op.sin2
        self.sout = self.op.sout
        plug(sin1, self.op.sin1)
        plug(sin2, self.op.sin2)


def stack_two_vectors(vec1, vec2, vec1_size, vec2_size, entityName=''):
    """
    ## This function stacks two vectors
    ## Input : Constant vector (not numpy arrays)
          : Constant vector (not numpy arrays)
          : size of first vector (int)
          : size of first vector (int)
    """
    op = Stack_of_vector(entityName)
    op.selec1(0, vec1_size)
    op.selec2(0, vec2_size)
    plug(vec1, op.signal('sin1'))
    plug(vec2, op.signal('sin2'))
    return op.signal('sout')


class StackZero(object):
    """
    This function stacks a zero before the vector
    Input : number of zeroes you want to stack up (int)
          : vector you want to stack the zeroes on top
          : size of vector (int)
    """

    def __init__(self, nb_0, vec_size, vec=None, prefix=''):
        self.zero = VectorConstant("zero")
        self.zero.sout.value = nb_0 * (0.,)

        self.op = Stack_of_vector(prefix + 'zero_stacker')
        self.op.selec1(0, nb_0)
        self.op.selec2(0, vec_size)
        plug(self.zero.sout, self.op.sin1)

        if vec is not None:
            plug(vec, self.op.sin2)

        self.sin = self.op.sin2
        self.sout = self.op.sout


def selec_vector(vec, start_index, end_index, entityName=''):
    """
    ## This function selects a part of the input vector (slices vector)
    ## Input : Constant vector (not numpy array)
         : start index (int)
         : end index (int)
    ## Ex : selec_vector([1,2,3,4], 1,3) = [2,3] {input must be a const vector}
    """
    op = Selec_of_vector(entityName)
    op.selec(start_index, end_index)
    plug(vec, op.signal('sin'))
    return op.sout


#
#  Math Operators
#


class Add2Vectors(object):
    """
    interface to the Add_of_vector entity
    """

    def __init__(self, vector_sin1, vector_sin2, entity_name=''):
        self.op = Add_of_vector(entity_name)
        self.sin = self.op.sin
        self.sout = self.op.sout
        plug(vector_sin1, self.op.sin(0))
        plug(vector_sin2, self.op.sin(1))


def add_vec_vec(vec1, vec2, entityName=''):
    """
    ## This function adds two Vectors
    ## Input : Constant vectors (not numpy arrays)
    """
    add = Add_of_vector(entityName)
    plug(vec1, add.signal('sin1'))
    plug(vec2, add.signal('sin2'))
    return add.sout


def subtract_vec_vec(pos1, pos2, entityName=''):
    """
    ## This function subtracts two Vectors
    ## Input : Constant vectors (not numpy arrays)
    """
    sub_op = Substract_of_vector(entityName)
    plug(pos1, sub_op.signal('sin1'))
    plug(pos2, sub_op.signal('sin2'))
    return sub_op.sout


def transpose_mat(mat, entityName=''):
    """
    ## This function transposes a matrix
    ## Input : Constant matrix (not numpy arrays)
    """
    op = MatrixTranspose(entityName)
    plug(mat, op.sin)
    return op.sout


def multiply_mat_vec(mat, vec, entityName=''):
    """
    ## This function multiplies a matrix and vector
    ## Input : Constant matrix (not numpy arrays)
             : Constant vector (not numpy array)
    """
    mat_mul = Multiply_matrix_vector(entityName)
    plug(mat, mat_mul.signal('sin1'))
    plug(vec, mat_mul.signal('sin2'))
    return mat_mul.sout


class MultiplyDoubleVector(object):
    """
    Simpler interface to multiply a double and a vector
    """

    def __init__(self, double_sin, vector_sin, entity_name=''):
        self.op = Multiply_double_vector(entity_name)
        self.sin1 = self.op.sin1
        self.sin2 = self.op.sin2
        self.sout = self.op.sout
        plug(double_sin, self.op.sin1)
        plug(vector_sin, self.op.sin2)


def mul_double_vec(doub, vec, entityName=''):
    """
    ## This function multiplies a double and vector
    ## Input : Constant double
             : Constant vector (not numpy array)
    """
    mul = Multiply_double_vector(entityName)
    mul.sin1.value = doub
    plug(vec, mul.signal('sin2'))
    return mul.sout

######################### Robotics operators ##################################


def hom2pos(robot_joint_signal, entityName=''):
    """
    ## This function transforms a homogenous matrix to xyz cordinate
    ## Input : robot (DynamicPinocchio model) joint signal
    """
    conv_pos = MatrixHomoToPose(entityName)
    plug(robot_joint_signal, conv_pos.signal('sin'))
    return conv_pos.signal('sout')
