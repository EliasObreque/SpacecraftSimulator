# -*- coding: utf-8 -*-

"""
* @file NormalRandom.py
* @author Gustavo Diaz H.
* @date 15 May 2020
* @brief Generic Normal Random Signal

* The signal is sampled from a normal gaussian distribution
* with zero mean and standard deviation provided
"""

import numpy as np


class NormalRandom(object):
    def __init__(self, std_dev):
        self.std_dev = std_dev[0] 						  #[unit]
        self.nr = np.random.normal(0, self.std_dev, 3)    #[unit]

    def __call__(self, *args, **kwargs):
        self.nr = np.random.normal(0, self.std_dev, 3)  # update normal random sample
        return self.nr