"""
@package py_dg_tools
@author Avadesh Meduri
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-03-01
@brief Contains functions that help the use of sot-core
"""


#################### Imports #################################################


import os, os.path

import numpy as np
import rospkg

from numbers import Number

from dynamic_graph_manager.dg_tools import (
    PoseQuaternionToPoseRPY,
    Division_of_double,
    Sin as SinEntity
)

from dynamic_graph import plug
from dynamic_graph.sot.core import Selec_of_vector
from dynamic_graph.sot.core.operator import *
from dynamic_graph.sot.core.math_small_entities import *
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.matrix_constant import MatrixConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double

################### Plug helper #############################################

# To track down implemenation bugs faster, check the arguments to `dg.plug` are
# actual signals. If not, raise an error.
#
# SEE: https://github.com/stack-of-tasks/dynamic-graph-python/issues/36

plug_ = dg.plug
def dg_plug_dbg(sig0, sig1):
    def assert_signal(sig):
        if not isinstance(sig, dg.signal_base.SignalBase):
            raise ValueError(
                'dynamic_graph.plug(): Passed in value is not a signal. sig=' +
                str(sig), sig)
    assert_signal(sig0)
    assert_signal(sig1)
    plug_(sig0, sig1)
dg.plug = dg_plug_dbg

################### Initialisers #############################################

def constDouble(val, entityName='', with_entity=False):
    """Creates a constant double value operator.

    Args:
        val: (double) Constant value to use
        entityName: (str, optional) Name of entity.
    Returns:
        Constant double signal.
    """
    sig = Add_of_double(entityName)
    sig.sin1.value = val
    sig.sin2.value = 0.
    if with_entity:
        return sig.sout, sig
    else:
        return sig.sout

def constVector(val, entityName=''):
    """
    ## This function initialises an constant vector
    ## Input : array (python list)
    """
    op = VectorConstant(entityName).sout
    op.value = list(val)
    return op

def vectorIdentity(vec1, vec_size, entityName):
    """Entity to label a vector signal."""
    add = Add_of_vector(entityName)
    plug(vec1, add.signal('sin1'))
    plug(constVector(vec_size * [0.,]), add.signal('sin2'))
    return VectorSignal(add.sout, vec_size)


def constMatrix(val, entityName):
    """
    ## This function initialises an constant matrix
    ## Input : matrix (python array)
    """
    op = MatrixConstant(entityName).sout
    op.value = val
    return op

################### OperatorSignals #############################################

import six
from dynamic_graph.signal_base import objectToString, SignalBase
class BaseOperatorSignal(SignalBase):
    def __init__(self, sig):
        super(BaseOperatorSignal, self).__init__(obj=sig.obj)

    def _op(self, other, op_name, op_maps):
        # TODO: If given other=signal, try to implicity convert to OperatorSignal
        #   by inferring the type from the signal name.
        #   Eg: other.name = 'SinEntity()::output(double)::sout'.

        for k, v in six.iteritems(op_maps):
            if isinstance(other, k):
                op = v['op']('')
                if 'value' in v:
                    op.signal(v['sinB']).value = other
                else:
                    dg.plug(other, op.signal(v['sinB']))
                dg.plug(self, op.signal(v['sinA']))
                return v['return_wrap'](self, op, other)

        raise ValueError('Unsupported "%s" operation with "%s"' % (op_name, str(other)))

    def _identity(self, entity_name):
        raise Exception('Identity operator not defined for this OperatorSignal type');

    def name_entity(self, entity_name):
        return self._identity(entity_name)

    def __add__(self, other):
        return self._op(other, '__add__', self._add)

    def __mul__(self, other):
        return self._op(other, '__mul__', self._mul)

    def __sub__(self, other):
        return self._op(other, '__sub__', self._sub)

    def __div__(self, other):
        return self.__truediv__(other)

    def __truediv__(self, other):
        return self._op(other, '__truediv__', self._truediv)

    def __repr__(self):
        return type(self).__name__ + ': value=' + str(self.value)

def opdef(op, return_wrap, use_value=False, use_sin1=True):
    """Creates a definition for how to use the operator.

    Args:
      op: Operator class to create.
      return_wrap: Function used to create the returned output.
      use_value: (optional) If provided, put the `other` as value
      use_sin1: If True, uses `sin1` and `sin2`, if False use `sin0`
          and `sin1` for the operator.
      """
    d = {
        'op': op,
        'return_wrap': return_wrap
    }
    if use_value:
        d['value'] = True
    if use_sin1:
        d['sinA'] = 'sin1'
        d['sinB'] = 'sin2'
    else:
        d['sinA'] = 'sin0'
        d['sinB'] = 'sin1'
    return d

class VectorSignal(BaseOperatorSignal):
    def __init__(self, sig, length=None):
        if isinstance(sig, list) or isinstance(sig, tuple) or isinstance(sig, np.ndarray):
            if length is None:
                length = len(sig)
            if len(sig) != length:
                raise ValueError('VectorSignal: Length wrong for provided vector.')
            sig = constVector(sig)

        if length is None:
            raise ValueError('VectorSignal: Need to provide length of vector signal.')

        super(VectorSignal, self).__init__(sig)
        self.length = length

        self._add = {
            VectorSignal: opdef(Add_of_vector, lambda s, op, other: VectorSignal(op.sout, s.length))
        }

        self._sub = {
            VectorSignal: opdef(Substract_of_vector, lambda s, op, other: VectorSignal(op.sout, s.length))
        }

    def _identity(self, entity_name):
        return vectorIdentity(self, self.length, entity_name)

    def __getitem__(self, val):
        if isinstance(val, slice):
            if val.step != 1 and val.step != None:
                raise ValueError('VectorSignal.__getitem__: Only support slice with step 1.')
            if val.start < 0 or val.start > self.length:
                raise ValueError('VectorSignal.__getitem__: Slice start index "%s" out of range' % (str(val.start)))
            if val.stop < 0 or val.stop > self.length:
                raise ValueError('VectorSignal.__getitem__: Slice stop index "%s" out of range.' % (str(val.start)))

            op = Selec_of_vector('')
            op.selec(val.start, val.stop)
            dg.plug(self, op.sin)
            return VectorSignal(op.sout, val.stop - val.start)
        elif isinstance(val, int):
            op = Component_of_vector('')
            op.setIndex(val)
            plug(self, op.sin)
            return DoubleSignal(op.sout)

        raise ValueError('VectorSignal.__getitem__: Unsupported item type "%s"' % (str(val)))

    def concat(self, other):
        if not isinstance(other, VectorSignal):
            raise ValueError('VectorSignal.concat(other): Expecting `other` to be of type VectorSignal but got "%s".' % str(other))

        op = Stack_of_vector("")
        op.selec1(0, self.length)
        op.selec2(0, other.length)
        plug(self, op.sin1)
        plug(other, op.sin2)
        return VectorSignal(op.sout, self.length + other.length)


class DoubleSignal(BaseOperatorSignal):
    def __init__(self, sig):
        if isinstance(sig, int) or isinstance(sig, float):
            sig = constDouble(sig)
        super(DoubleSignal, self).__init__(sig)

        return_wrap = lambda s, op, other: DoubleSignal(op.sout)

        self._add = {
            float: opdef(Add_of_double, return_wrap, use_value=True),
            DoubleSignal: opdef(Add_of_double, return_wrap)
        }

        self._sub = {
            float: opdef(Substract_of_double, return_wrap, use_value=True),
            DoubleSignal: opdef(Substract_of_double, return_wrap)
        }

        self._mul = {
            float: opdef(Multiply_of_double, return_wrap, use_value=True, use_sin1=False),
            DoubleSignal: opdef(Multiply_of_double, return_wrap, use_sin1=False),
            VectorSignal: opdef(Multiply_double_vector, lambda s, op, other: VectorSignal(op.sout, other.length))
        }

        self._truediv = {
            float: opdef(Division_of_double, return_wrap, use_value=True),
            DoubleSignal: opdef(Division_of_double, return_wrap)
        }

    def vector(self):
        return self * VectorSignal([1.])


#################### Operators ################################################


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


def selec_vector(vec, start_index, end_index, entityName=''):
    """
    ## This function selects a part of the input vector (slices vector)
    ## Input : Constant vector (not numpy array)
         : start index (int)
         : end index (int)
    ## Ex    : selec_vector([1,2,3,4], 1,3) = [2,3] {input must be a const vector}
    """
    op = Selec_of_vector(entityName)
    op.selec(start_index, end_index)
    plug(vec, op.signal('sin'))
    return op.sout


def component_of_vector(vector, index, entityName):
    """
    ## This function selects a compnent of the input vector
    ## Input : Constant vector (not numpy array)
         : index (int)
    """
    comp_of_vect = Component_of_vector(entityName)
    comp_of_vect.setIndex(index)
    plug(vector, comp_of_vect.sin)
    return comp_of_vect.sout


def sin_doub(db1, entityName=''):
    op = SinEntity(entityName)
    plug(db1, op.sin)
    return op.sout


###################  Math Operators ##########################################


def add_doub_doub(db1, db2, entityName):
    """
    ## This function adds two doubles
    ## Input : db1 - double (number)
             : db2 - double (value)
    """
    add = Add_of_double(entityName)
    add.sin1.value = db1
    add.sin2.value = db2
    return add


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


def multiply_mat_vec(mat,vec, entityName=''):
    """
    ## This function multiplies a matrix and vector
    ## Input : Constant matrix (not numpy arrays)
             : Constant vector (not numpy array)
    """
    mat_mul = Multiply_matrix_vector(entityName)
    plug(mat, mat_mul.signal('sin1'))
    plug(vec, mat_mul.signal('sin2'))
    return mat_mul.sout


def mul_double_vec(doub, vec, entityName=''):
    """
    ## This function multiplies a double and vector
    ## Input : Double value or signal
             : Vector signal
    """
    mul = Multiply_double_vector(entityName)
    if isinstance(doub, Number):
        mul.sin1.value = doub
    else:
        plug(doub, mul.sin1)
    plug(vec, mul.signal('sin2'))
    return mul.sout


def mul_vec_vec(vec1, vec2, entityName):
    """
    ## This function multiplies two Vectors element wise
    ## Input : Constant vectors (not numpy arrays)
    """
    vec_mul = Multiply_of_vector(entityName)
    plug(vec1, vec_mul.sin0)
    plug(vec2, vec_mul.sin1)
    return vec_mul.sout


def div_doub_doub(db1, db2, entityName=''):
    div = Division_of_double(entityName)
    plug(db1, div.sin1)
    plug(db2, div.sin2)
    return div.sout

######################### Robotics operators ##################################


def hom2pos(robot_joint_signal, entityName=''):
    """
    ## This function transforms a homogenous matrix to xyz cordinate
    ## Input : robot (DynamicPinocchio model) joint signal
    """
    conv_pos = MatrixHomoToPose(entityName)
    plug(robot_joint_signal, conv_pos.signal('sin'))
    return conv_pos.signal('sout')


def convert_quat_se3(quat, entityName):
    """
    ## This function transforms a quaternion(4d) to se3
    ## Input : quaternion signal
    """

    quat_to_se3 = QuaternionToMatrix(entityName)
    plug(quat, quat_to_se3.signal('sin'))
    return quat_to_se3.signal('sout')


def basePoseQuat2PoseRPY(q_base, entityName=''):
    op = PoseQuaternionToPoseRPY(entityName)
    plug(q_base, op.sin)
    return op.sout


######################### Standard vectors ####################################


def zero_vec(vec_size, entityName):
    """
    ## This function creates a zero constvector of vec_size
    ## Input : size of zero vector (int)
    """
    zero_vec = np.zeros(vec_size)
    return constVector(zero_vec, entityName)
