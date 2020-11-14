
"""
Created on Fri Oct 29 10:52:34 2020

@author: Elias Obreque
els.obrq@gmial.com
"""

import numpy as np
from Library.math_sup.Quaternion import Quaternions
from Dynamics.ClockGenerator import ClockGenerator
from Spacecraft.SubSystems import SubSystems

twopi = 2.0 * np.pi
deg2rad = np.pi / 180.0
rad2deg = 1 / deg2rad


class GroundStation(object):
    def __init__(self, properties, dynamics, simtime):
        self.simtime = simtime
        self.dynamics_gs = dynamics

        self.select_gs = {}
        self.gs_name = []
        print("-----------------------------")
        print('Ground Station:')
        for i in range(len(properties)):
            self.gs_name.append(properties[i]['gs_name'])
            self.select_gs[properties[i]['gs_name']] = {'alt': properties[i]['alt'],
                                                        'long': deg2rad*properties[i]['long'],
                                                        'lat': deg2rad*properties[i]['lat']}
            print(' - ' + properties[i]['gs_name'] + ': ' + 'Altitude: ' + str(properties[i]['alt']) +
                  ' [m] - Longitude: ' + str(properties[i]['long']) + ' [°] - Latitude: ' + str(properties[i]['lat']) + ' [°]')

        if dynamics.ephemeris.center_object == 'EARTH':
            self.e2   = dynamics.ephemeris.selected_body['EARTH'].e2
            self.a_axis = dynamics.ephemeris.selected_body['EARTH'].radiuskm * 1e3
            self.geodetic2ecef()

    def update(self):
        # Dynamics updates
        self.coverage_zone()
        return

    def coverage_zone(self):
        return

    def geodetic2ecef(self):
        for gs_n in self.gs_name:
            # geodetic to geocentric
            mu = self.select_gs[gs_n]['lat']
            N = self.a_axis / np.sqrt(1 - self.e2 ** 2 * np.sin(mu) ** 2)
            rho = (N + self.select_gs[gs_n]['alt']) * np.sin(mu)
            z = (N * (1 - self.e2) + self.select_gs[gs_n]['alt']) * np.sin(mu)
            self.select_gs[gs_n]['lat'] = np.pi/2 + np.arctan2(rho, z)
            # geocentric to ECEF
            radius = self.a_axis + self.select_gs[gs_n]['alt']
            self.select_gs[gs_n]['x_ecef'] = radius * np.cos(self.select_gs[gs_n]['lat']) * np.cos(
                self.select_gs[gs_n]['long'])
            self.select_gs[gs_n]['y_ecef'] = radius * np.cos(self.select_gs[gs_n]['lat']) * np.sin(
                self.select_gs[gs_n]['long'])
            self.select_gs[gs_n]['z_ecef'] = radius * np.sin(self.select_gs[gs_n]['lat'])
        return

    def create_report(self):
        return
