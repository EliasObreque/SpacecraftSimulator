# -*- coding: utf-8 -*-
"""
Created on Fri Jan 17 03:00:38 2020

@author: EO
"""

import numpy as np
from .EarthCenter import EarthCenterOrbit
from Library.math_sup.tools_reference_frame import fmod2
from .TwoBodyProblem import TwoBodyProblem

twopi = 2.0 * np.pi
deg2rad = np.pi / 180.0
rad2deg = 1 / deg2rad


class MainOrbit(object):
    def __init__(self, orbit_spacecraft, step_width, selected_planet):
        self.step_width             = step_width
        self.selected_planet        = selected_planet
        self.orbit_properties       = orbit_spacecraft['Orbit_info']
        self.propagation_properties = orbit_spacecraft['propagate']
        self.current_position_i       = np.zeros(3)
        self.current_velocity_i       = np.zeros(3)
        self.propagator_model       = None
        self.wgs                    = self.propagation_properties['wgs']
        self.current_velocity_b     = np.zeros(3)
        self.current_lat            = 0
        self.current_long           = 0
        self.current_alt            = 0
        self.historical_position_i  = []
        self.historical_velocity_i  = []
        self.historical_lats        = []
        self.historical_longs       = []
        self.historical_alts        = []
        self.acc_i                  = np.array([0, 0, 0])

    def set_propagator(self):
        if self.propagation_properties['propagate_mode'] == 0:
            self.propagator_model = TwoBodyProblem(self.selected_planet.mu,
                                                   self.step_width,
                                                   self.current_position_i,
                                                   self.current_velocity_i)
        elif self.propagation_properties['propagate_mode'] == 1:
            line1      = self.orbit_properties[0]
            line2      = self.orbit_properties[1]
            self.propagator_model = EarthCenterOrbit(line1, line2, self.wgs)
        elif self.propagation_properties['propagate_mode'] == 2:
            print('2')
        elif self.propagation_properties['propagate_mode'] == 3:
            print('3')

    def update_orbit(self, current_jd):
        self.current_position_i, self.current_velocity_i = self.propagator_model.update_state(current_jd)

    def update_attitte(self, q_i2b):
        self.current_velocity_b = q_i2b.frame_conv(self.current_velocity_i)

    def add_ext_force_b(self, force_b, q_b2i, mass):
        force_i = q_b2i.frame_conv(force_b)
        self.propagator_model.add_force_i(force_i, mass)

    def add_int_force_b(self, force_b, q_b2i, mass):
        force_i = q_b2i.frame_conv(force_b)
        self.propagator_model.add_force_i(force_i, mass)

    def get_velocity_b(self):
        return self.current_velocity_b

    def TransECItoGeo(self):
        current_sideral = self.selected_planet.get_current_sideral()
        r = np.sqrt(self.current_position_i[0] ** 2 + self.current_position_i[1] ** 2)

        long = fmod2(np.arctan2(self.current_position_i[1], self.current_position_i[0]) - current_sideral)
        lat = np.arctan2(self.current_position_i[2], r)

        flag_iteration = True

        while flag_iteration:
            phi = lat
            c = 1 / np.sqrt(1 - self.propagator_model.e2 * np.sin(phi) * np.sin(phi))
            lat = np.arctan2(self.current_position_i[2] + self.propagator_model.radiusearthkm * c
                             * self.propagator_model.e2 * np.sin(phi) * 1000, r)
            if (np.abs(lat - phi)) <= self.propagator_model.tolerance:
                flag_iteration = False

        alt = r / np.cos(lat) - self.propagator_model.radiusearthkm * c * 1000  # *metros
        if lat > np.pi / 2:
            lat -= twopi
        self.current_alt = alt
        self.current_lat = lat
        self.current_long = long
        return lat, long, alt

    def save_orbit_data(self):
        self.historical_position_i.append(self.current_position_i)
        self.historical_velocity_i.append(self.current_velocity_i)
        self.historical_lats.append(self.current_lat)
        self.historical_longs.append(self.current_long)
        self.historical_alts.append(self.current_alt)

    def get_log_values(self):
        report_orbit = {'sat_position_i(X)[m]': np.array(self.historical_position_i)[:, 0],
                        'sat_position_i(Y)[m]': np.array(self.historical_position_i)[:, 1],
                        'sat_position_i(Z)[m]': np.array(self.historical_position_i)[:, 2],
                        'sat_velocity_i(X)[m/s]': np.array(self.historical_velocity_i)[:, 0],
                        'sat_velocity_i(Y)[m/s]': np.array(self.historical_velocity_i)[:, 1],
                        'sat_velocity_i(Z)[m/s]': np.array(self.historical_velocity_i)[:, 2],
                        'lat[rad]': np.array(self.historical_lats),
                        'lon[rad]': np.array(self.historical_longs),
                        'alt[m]': np.array(self.historical_alts)}
        return report_orbit

