
import numpy as np
from Library.igrf.IGRF import calculate_igrf
from Library.math_sup.tools_reference_frame import rotationY, rotationZ

RAD2DEG = 180/np.pi
DEG2RAD = 1/RAD2DEG


class MagEnv(object):
    def __init__(self, mag_properties):
        self.Mag_i = np.zeros(3)
        self.Mag_b = np.zeros(3)
        self.calcmagflag = True
        self.envir_flag = mag_properties['mag_calculation']

    def update(self, dynamics, decyear, sideral=None, geocen=None):
        if sideral is None:
            sideral  = dynamics.ephemeris.selected_body['EARTH'].current_sideral
        if geocen is None:
            lat = dynamics.orbit.current_lat
            lon = dynamics.orbit.current_long
            alt = dynamics.orbit.current_alt
        else:
            lat = geocen[0]
            lon = geocen[1]
            alt = geocen[2]
        q_i2b = dynamics.attitude.current_quaternion_i2b
        self.calc_mag(decyear, sideral, lat, lon, alt, q_i2b)

    def calc_mag(self, decyear, sideral, lat, lon, alt, q_i2b):
        if not self.calcmagflag:
            return
        alt /= 1000
        """
         itype = 1 if geodetic(spheroid)
         itype = 2 if geocentric(sphere)
         alt   = height in km above sea level if itype = 1
               = distance from centre of Earth in km if itype = 2 (>3485 km)
        """
        x, y, z, f, gccolat = calculate_igrf(0, decyear, alt, lat, lon, itype=1)
        mag_local = [x, y, z]

        self.mag_NED_to_ECI(mag_local, gccolat, lon, sideral)

        self.Mag_b = q_i2b.frame_conv(self.Mag_i)

    def add_mag_noise(self):
        return

    def mag_NED_to_ECI(self, mag_0, theta, lonrad, gmst):
        mag_local_0y = rotationY(mag_0, np.pi - theta)
        mag_local_yz = rotationZ(mag_local_0y, -lonrad)
        self.Mag_i   = rotationZ(mag_local_yz, -gmst)

    def get_mag_b(self):
        return self.Mag_b