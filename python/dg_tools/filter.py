#!/usr/bin/env python

# @namespace dg_tools.filter
""" Filter entities factory

    @file
    @copyright Copyright (c) 2017-2019,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

# Python 3 compatibility, has to be called just after the hashbang.
from __future__ import print_function, division
import os
from scipy.signal import butter
from dynamic_graph.sot.core.filter_differentiator import FilterDifferentiator


class ButterWorthFilter(object):
    """  Butterworth filter implementation in dynamic graph.

    Computes the Butterworth filter coefficient using scipy and create the
    appropriate dynamic graph filter.

    Attributes:
        filter: Dynamic graph entity implementing an infinite impedance filter.

    """

    def __init__(self, name=""):
        """ Constructor 

        Args:
            name:
        """
        self.name = name

        self.filter = FilterDifferentiator(self.name)
        self.sin = self.filter.x
        self.sout = self.filter.x_filtered

    def init(self, size_of_input, control_time_step, percentage_nyquist_cutoff,
             filter_order):
        """ Initialize the filter using scipy.

        Args:
            size_of_input:
            control_time_step:
            percentage_nyquist_cutoff:
            filter_order:
            prefix:

        """
        # copying the arguments internally
        self.size_of_input = size_of_input
        self.control_time_step = control_time_step
        self.percentage_nyquist_cutoff = percentage_nyquist_cutoff
        self.filter_order = filter_order
        # filter paramters
        self.numerator = []
        self.denominator = []
        self._compute_numerator_denominator()
        # initialize the entity
        self.filter.init(control_time_step, size_of_input,
                         self.numerator, self.denominator)

    def update(self, filter_order, percentage_nyquist_cutoff):
        self.percentage_nyquist_cutoff = percentage_nyquist_cutoff
        self.filter_order = filter_order
        self._compute_numerator_denominator()

    def _compute_numerator_denominator(self):
        self.numerator, self.denominator = butter(
            self.filter_order, self.percentage_nyquist_cutoff,
            btype='low', output='ba')
