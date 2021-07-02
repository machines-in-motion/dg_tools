"""
@package dg_blmc_robots
@file dg_solo.py
@author Avadesh Meduri
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-02-06
@brief This file contains dynamic graph tools specific to the robot Solo
"""

from pinocchio.utils import zero
import numpy as np
import string
from dynamic_graph import plug
from dynamic_graph.sot.core.math_small_entities import (
    Component_of_vector,
    Selec_of_vector,
    Multiply_double_vector,
    Add_of_vector,
)
from dynamic_graph.sot.core.filter_differentiator import FilterDifferentiator


class Sliders(object):
    """
    This class filter the sliders signals from the hardware.

    Filter paramters:
    - order =  6
    - fs =  1000.0
    - nyq =  500.0
    - low =  0.1
    - lowcut =  50.0
    - numerator =  [8.57655707e-06, 5.14593424e-05, 1.28648356e-04,
                    1.71531141e-04, 1.28648356e-04, 5.14593424e-05,
                    8.57655707e-06]
    - denominator =  [1., -4.7871355, 9.64951773, -10.46907889,   6.44111188,
                      -2.12903875,   0.29517243]
    """

    def __init__(self, nb_sliders, prefix="", control_time_step=0.001):
        """
        Initialize the class by creating all the output signals
        """
        assert nb_sliders > 0
        assert type(nb_sliders) is int

        self.nb_sliders = nb_sliders
        self.prefix = prefix
        self.control_time_step = control_time_step

        # Define the filter
        self.numerator = np.array([
            8.57655707e-06,
            5.14593424e-05,
            1.28648356e-04,
            1.71531141e-04,
            1.28648356e-04,
            5.14593424e-05,
            8.57655707e-06,
        ])
        self.denominator = np.array([
            1.0,
            -4.7871355,
            9.64951773,
            -10.46907889,
            6.44111188,
            -2.12903875,
            0.29517243,
        ])
        self.sliders_filtered = FilterDifferentiator(
            self.prefix + "_lowpass_filter"
        )
        self.sliders_filtered.init(
            self.control_time_step,
            self.nb_sliders,
            self.numerator,
            self.denominator,
        )

        #
        # Input signal of the subgraph
        #
        self.sin = self.sliders_filtered.x

        #
        # Slider operations
        #

        for i, slider_letter in enumerate(
            list(string.ascii_uppercase[:nb_sliders])
        ):
            # names
            slider_name = "slider_" + slider_letter
            slider_component = slider_name + "_component_of_vector_sout"
            slider_sel_vec_sout = slider_name + "_selec_of_vector_sout"
            slider_sel_vec = slider_name + "_selec_of_vector"
            slider_scale = slider_name + "_scale"
            slider_offset = slider_name + "_offset"

            # Get the slider as a vector
            self.__dict__[slider_sel_vec] = Selec_of_vector(
                self.prefix + slider_sel_vec
            )
            self.__dict__[slider_sel_vec].selec(i, i + 1)
            plug(
                self.sliders_filtered.x_filtered,
                self.__dict__[slider_sel_vec].sin,
            )

            # offset the slider
            self.__dict__[slider_offset] = Add_of_vector(
                self.prefix + slider_offset
            )
            self.__dict__[slider_offset].setSignalNumber(2)

            # sin1 - sin2
            plug(
                self.__dict__[slider_sel_vec].sout,
                self.__dict__[slider_offset].sin(0),
            )
            self.__dict__[slider_offset].sin(1).value = np.array([0.0])

            # Scale the slider
            self.__dict__[slider_scale] = Multiply_double_vector(
                self.prefix + slider_scale
            )
            self.__dict__[slider_scale].sin1.value = 1.0
            plug(
                self.__dict__[slider_offset].sout,
                self.__dict__[slider_scale].sin2,
            )

            # Get the slider as a double
            self.__dict__[slider_component] = Component_of_vector(
                self.prefix + slider_component
            )
            self.__dict__[slider_component].setIndex(0)
            plug(
                self.__dict__[slider_scale].sout,
                self.__dict__[slider_component].sin,
            )

            # Get the slider as a vector
            self.__dict__[slider_sel_vec_sout] = Selec_of_vector(
                self.prefix + slider_sel_vec_sout
            )
            self.__dict__[slider_sel_vec_sout].selec(0, 1)
            plug(
                self.__dict__[slider_scale].sout,
                self.__dict__[slider_sel_vec_sout].sin,
            )

            # create some shortcut
            class Slider(object):
                """
                empty shell filled below
                """

                pass

            self.__dict__[slider_name] = Slider()
            self.__dict__[slider_name].double = self.__dict__[slider_component]
            self.__dict__[slider_name].vector = self.__dict__[slider_scale]
            self.__dict__[slider_letter] = self.__dict__[slider_component].sout
            self.__dict__[slider_letter + "_vec"] = self.__dict__[
                slider_sel_vec_sout
            ].sout

    def set_scale_values(self, scale_values):
        assert len(scale_values) == self.nb_sliders

        for slider_letter, scale in zip(
            list(string.ascii_uppercase[: self.nb_sliders]), scale_values
        ):
            # names
            slider_name = "slider_" + slider_letter
            slider_scale = slider_name + "_scale"
            # set the scale values
            self.__dict__[slider_scale].sin1.value = scale

    def set_offset_values(self, offset_values):
        assert len(offset_values) == self.nb_sliders

        for slider_letter, offset in zip(
            list(string.ascii_uppercase[: self.nb_sliders]), offset_values
        ):
            # names
            slider_name = "slider_" + slider_letter
            slider_offset = slider_name + "_offset"
            # set the offset
            offset_val = zero(1)
            offset_val[:] = offset
            self.__dict__[slider_offset].sin(1).value = offset_val

    def zero_slider(self):
        self.set_offset_values(-self.sin.value)

    def plug_slider_signal(self, slider_positions_sig):
        plug(slider_positions_sig, self.sin)

    def trace(self, robot):
        robot.add_trace(self.sliders_filtered.name, "x_filtered")
        for slider_letter in list(string.ascii_uppercase[: self.nb_sliders]):
            # names
            slider_name = "slider_" + slider_letter
            slider_component = slider_name + "_Component_of_vector"
            slider_component_entity_name = self.prefix + slider_component
            robot.add_trace(slider_component_entity_name, "sout")
