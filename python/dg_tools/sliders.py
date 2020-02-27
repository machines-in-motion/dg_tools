"""
@package dg_blmc_robots
@file dg_solo.py
@author Avadesh Meduri
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-02-06
@brief This file contains dynamic graph tools specific to the robot Solo
"""


import string
from dynamic_graph import plug
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
from dynamic_graph.sot.core.math_small_entities import (
    Component_of_vector,
    Selec_of_vector,
    Multiply_double_vector,
    Add_of_vector
)

from dg_tools.filter import ButterWorthFilter

from dg_tools.utils import VectorSignal, DoubleSignal

class Sliders(object):
    """
    This class filter the sliders signals from the hardware.
    """

    def __init__(self, nb_sliders, prefix="", filter_size=400, control_time_step=0.001,
                 percentage_nyquist_cutoff=0.1, filter_order=6):
        """
        Initialize the class by creating all the output signals
        """
        assert nb_sliders > 0
        assert type(nb_sliders) is int

        self.nb_sliders = nb_sliders
        self.prefix = prefix

        self.sliders_filtered = ButterWorthFilter(self.prefix + "_sliders_butter")
        self.sliders_filtered.init(nb_sliders, control_time_step,
            percentage_nyquist_cutoff, filter_order)
        self.sin = self.sliders_filtered.sin

        #
        # Slider operations
        #

        for i, slider_letter in enumerate(
                list(string.ascii_uppercase[:nb_sliders])):
            # names
            slider_name = "slider_" + slider_letter
            slider_component = slider_name + "_Component_of_vector"
            slider_sel_vec = slider_name + "_selec_of_vector"
            slider_scale = slider_name + "_scale"
            slider_offset = slider_name + "_offset"

            # Get the slider as a vector
            self.__dict__[slider_sel_vec] = Selec_of_vector(
                self.prefix + slider_sel_vec)
            self.__dict__[slider_sel_vec].selec(i, i + 1)
            plug(self.sliders_filtered.sout,
                 self.__dict__[slider_sel_vec].sin)

            # offset the slider
            self.__dict__[slider_offset] = Add_of_vector(
                self.prefix + slider_offset)
            # sin1 - sin2
            plug(self.__dict__[slider_sel_vec].sout,
                 self.__dict__[slider_offset].sin1)
            self.__dict__[slider_offset].sin2.value = [0.0]

            # Scale the slider
            self.__dict__[slider_scale] = Multiply_double_vector(
                self.prefix + slider_scale)
            self.__dict__[slider_scale].sin1.value = 1.0
            plug(self.__dict__[slider_offset].sout,
                 self.__dict__[slider_scale].sin2)

            # Get the slider as a double
            self.__dict__[slider_component] = Component_of_vector(
                self.prefix + slider_component)
            self.__dict__[slider_component].setIndex(0)
            plug(self.__dict__[slider_scale].sout,
                 self.__dict__[slider_component].sin)

            # create some shortcut
            class Slider(object):
                """
                empty shell filled below
                """
                pass
            self.__dict__[slider_name] = Slider()
            self.__dict__[slider_name].double = self.__dict__[slider_component]
            self.__dict__[slider_name].vector = self.__dict__[slider_scale]
            self.__dict__[slider_letter] = DoubleSignal(self.__dict__[slider_component].sout)
            self.__dict__[slider_letter + "_vec"] = VectorSignal(self.__dict__[slider_scale].sout, 1)


    def set_scale_values(self, scale_values):
        assert len(scale_values) == self.nb_sliders

        for slider_letter, scale in zip(
                list(string.ascii_uppercase[:self.nb_sliders]),
                scale_values):
            # names
            slider_name = "slider_" + slider_letter
            slider_scale = slider_name + "_scale"
            # set the scale values
            self.__dict__[slider_scale].sin1.value = scale

    def set_offset_values(self, offset_values):
        assert len(offset_values) == self.nb_sliders

        for slider_letter, offset in zip(
                list(string.ascii_uppercase[:self.nb_sliders]),
                offset_values):
            # names
            slider_name = "slider_" + slider_letter
            slider_offset = slider_name + "_offset"
            # set the offset
            self.__dict__[slider_offset].sin2.value = [offset]

    def plug_slider_signal(self, slider_positions_sig):
        plug(slider_positions_sig, self.sin)

    def trace(self, robot):
        robot.add_trace(self.sliders_filtered.name, "x_filtered")
        for i, slider_letter in enumerate(
                list(string.ascii_uppercase[:self.nb_sliders])):
            # names
            slider_name = "slider_" + slider_letter
            slider_component = slider_name + "_Component_of_vector"
            slider_component_entity_name = self.prefix + slider_component
            robot.add_trace(slider_component_entity_name, "sout")
