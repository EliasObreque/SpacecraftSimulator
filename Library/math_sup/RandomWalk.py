# -*- coding: utf-8 -*-

"""
* @file RandomWalk.py
* @author Gustavo Diaz H.
* @date 15 May 2020
* @brief Generic Random Walk process

* The signal rate is modeled as white gaussian noise
* and is solve using clasic Runge-Kutta method RK4
"""

import numpy as np


class RandomWalk(object):
    def __init__(self, step_width, std_dev, limit):
        self.step_width = step_width 					  #[s]
        self.std_dev = std_dev[0] 						  #[unit]
        self.limit = limit[0] 							  #[unit]
        self.rn = np.random.normal(0, self.std_dev, 3)    #[unit]
        self.tn = np.zeros(3)                             #[s]

    def __call__(self, *args, **kwargs):
        self.randomWalkRungeKutta()
        return self.rn

    def rwRate(self, tn, rn):
        """
        * The rate of change of the random walk process as funtion of time and the state.
        * It's modeled as white gausian noise in such a way that it stays in a given range
        * Mathematically: d(rw)/dt = rwRate(t, rw)  -->[dy/dt = f(t, y)]
        *
        * @param tn Float Time at which is the process is evaluated/sampled
        * @param rn Float Random walk state at wich evaluate the rate
        * @return rate Float Actual rate at given time and state
        """
        nrs = np.random.normal(0, self.std_dev, 3)  # update normal random sample
        rate = np.zeros(3)
        for i in range(0,3):
            if rn[i]>self.limit:
                rate[i] = -abs(nrs[i])
            elif rn[i]<-self.limit:
                rate[i] = abs(nrs[i])
            else:
                rate[i] = nrs[i]
        return rate

    def randomWalkRungeKutta(self):
        """
        * Runge-Kutta method to solve the random walk diferential equation
        *
        * @update self.rn np.array((3,1)) Actual value of the solved random walk process
        * @update self.tn np.array((3,1)) Time at which is solved the actual random walk process
        """
        k1 = self.rwRate(self.tn, self.rn)
        k2 = self.rwRate(self.tn + 0.5*self.step_width, self.rn + 0.5*self.step_width*k1)
        k3 = self.rwRate(self.tn + 0.5*self.step_width, self.rn + 0.5*self.step_width*k2)
        k4 = self.rwRate(self.tn + self.step_width, self.rn + self.step_width*k3)

        self.rn = self.rn + self.step_width*(k1 + 2*(k2+k3)+k4)/6.0
        self.tn = self.tn + self.step_width