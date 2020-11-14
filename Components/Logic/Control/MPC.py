"""
Created by:

@author: Elias Obreque
@Date: 11/13/2020 9:25 PM 
els.obrq@gmail.com

"""
import numpy as np
from copy import deepcopy


class MPC(object):
    def __init__(self, dynamics, controller_parameters):
        self.mpc_dynamics = deepcopy(dynamics)
