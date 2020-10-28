# -*- coding: utf-8 -*-
"""
Created on Thu Jan 16 05:54:44 2020

@author: EO
#satrec.inclo: Inclination [rad]
#satrec.nodeo: Right Ascension of the Ascending Node [rad]
#satrec.argpo: Argument of Perigee [rad]
#satrec.mo   : Mean Anomaly [rad]
#satrec.ecco : Eccentricity [-]
#satrec.no: Mean Motion [rad/min]

"""

from Library.sgp4.earth_gravity import wgs72, wgs72old, wgs84
from Library.sgp4.io import twoline2rv


import numpy as np

twopi = 2.0 * np.pi
deg2rad = np.pi / 180.0
rad2deg = 1 / deg2rad


class EarthCenterOrbit(object):

    def __init__(self,
                 line1,
                 line2,
                 wgs):
        self.line1 = line1
        self.line2 = line2
        self.position_i = np.zeros(3)
        self.velocity_i = np.zeros(3)
        self.acc_i = np.zeros(3)

        if wgs == 0:
            self.wgs = wgs72old
            self.radiusearthkm = 6378.135  # km
            self.f = 1.0 / 298.26
        elif wgs == 1:
            self.wgs = wgs72
            self.radiusearthkm = 6378.135  # km
            self.f = 1.0 / 298.26
        elif wgs == 2:
            self.wgs = wgs84
            self.radiusearthkm = 6378.137  # km
            self.f = 1.0 / 298.257223563

        self.e2 = self.f * (2 - self.f)
        self.satellite = twoline2rv(self.line1, self.line2, self.wgs)
        self.tolerance = 1e-10  # rad

    def update_state(self, current_jd):
        position_i, velocity_i = self.satellite.propagate(j=current_jd)
        self.position_i = np.array(position_i) * 1000
        self.velocity_i = np.array(velocity_i) * 1000
        return self.position_i, self.velocity_i  # [m]

    def add_force_i(self, force_i, mass):
        self.acc_i *= 0
