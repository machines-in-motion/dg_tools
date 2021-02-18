#!/usr/bin/env python

# @namespace dg_tools.demos
""" Filter entities demos

    @file
    @copyright Copyright (c) 2017-2019,
               New York University and Max Planck Gesellschaft,
               License BSD-3-Clause
"""

# Python 3 compatibility, has to be called just after the hashbang.
from __future__ import print_function, division

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
from dg_tools.filter import ButterWorthFilter

if __name__ == "__main__":

    my_filter = ButterWorthFilter('butterworth_filter')

    # Sample rate and desired cutoff frequencies (in Hz).
    order = 6
    fs = 1000.0
    nyq = 0.5 * fs
    low = 0.1
    lowcut = low * nyq

    print("Filter paramters:")
    print("- order = ", order)
    print("- fs = ", fs)
    print("- nyq = ", nyq)
    print("- low = ", low)
    print("- lowcut = ", lowcut)

    my_filter.init(1, 1.0/fs, low, order)

    my_filter.update(low, order)

    print("- numerator = ", my_filter.numerator)
    print("- denominator = ", my_filter.denominator)

    # Filter a noisy signal.
    T = 0.2
    nsamples = int(T * fs)
    t = np.linspace(0, T, nsamples, endpoint=False)
    a = [0.1, 0.005, 0.004, 0.003]
    
    f0 = 50.0
    x = a[0] * np.sin(2 * np.pi * 1.2 * np.sqrt(t))
    x += a[1] * np.cos(2 * np.pi * 312 * t + 0.1)
    x += a[2] * np.cos(2 * np.pi * f0 * t + .11)
    x += a[3] * np.cos(2 * np.pi * 2000 * t)

    plt.figure(1)
    plt.clf()
    plt.plot(t, x, label='Noisy signal')

    y = []
    for i in range(x.size):
        my_filter.sin.value = np.array([x[i]])
        my_filter.sout.recompute(i)
        y += [my_filter.sout.value]

    plt.plot(t, y, label='Filtered signal (%g Hz)' % f0)
    plt.xlabel('time (seconds)')
    # plt.hlines([-np.sum(a), np.sum(a)], 0, T, linestyles='--')
    plt.grid(True)
    plt.axis('tight')
    plt.legend(loc='upper left')

    plt.show()
