# -*- coding: utf-8 -*-
"""
Created on Thu Jan 16 10:52:34 2020

@author: EO
"""
import numpy as np
from .SubSystems import SubSystems
from Dynamics.Dynamics import Dynamics
from Dynamics.ClockGenerator import ClockGenerator


class Spacecraft(SubSystems):
    def __init__(self, dynamics_properties, components_properties, simtime):

        self.simtime = simtime
        self.dynamics = Dynamics(dynamics_properties, self.simtime)
        self.isDark = False
        self.master_data_satellite = {}
        print('Spacecraft name: ' + str(dynamics_properties['Attitude']['spacecraft_name']))
        # Add components
        print('Spacecraft components:')
        SubSystems.__init__(self, components_properties, self.dynamics, self.simtime.stepsimTime)
        self.clockgenerator = ClockGenerator(self.subsystems, self.system_name)

    def update(self):
        # Dynamics updates
        self.dynamics.update()
        self.shadow_zone()
        # Tick the time on component
        for i_ in range(int(self.simtime.stepsimTime*1000)):
            self.clockgenerator.tick_to_components(self.isDark)
        return

    def update_data(self):
        # Historical data components
        self.save_log_values()
        self.dynamics.attitude.save_attitude_data()
        self.dynamics.orbit.save_orbit_data()
        self.dynamics.ephemeris.save_ephemeris_data()
        self.simtime.save_simtime_data()
        # Historical data subsystems
        for subsys in self.system_name:
            if self.subsystems[subsys] is not None:
                self.subsystems['ADCS'].save_data()

    def shadow_zone(self):
        sun_pos_from_center_i = self.dynamics.ephemeris.selected_body['SUN'].get_pos_from_center_i()
        center_object_radius = self.dynamics.ephemeris.selected_body[self.dynamics.ephemeris.center_object].radiuskm
        center_object_radius *= 1000    # meters
        sc_pos_from_center_i = self.dynamics.orbit.current_position_i
        point_prodcut = np.dot(sun_pos_from_center_i, sc_pos_from_center_i)
        r_sun = np.linalg.norm(sun_pos_from_center_i)
        r_sc  = np.linalg.norm(sc_pos_from_center_i)
        theta = np.arccos(point_prodcut/(r_sc * r_sun))
        theta_sun = np.arccos(center_object_radius / r_sun)
        theta_sc = np.arccos(center_object_radius / r_sc)
        if theta_sc + theta_sun < theta:
            # Shadow
            self.isDark = True
        else:
            self.isDark = False

    def create_report(self):
        report_attitude = self.dynamics.attitude.get_log_values()
        report_orbit = self.dynamics.orbit.get_log_values()
        report_ephemerides = self.dynamics.ephemeris.get_log_values()
        report_timelog = self.simtime.get_log_values()
        report_subsystems = {}

        for subsys in self.system_name:
            if self.subsystems[subsys] is not None:
                report_subsystems = {**report_subsystems,
                                     **self.subsystems[subsys].get_log_values(subsys)}

        self.master_data_satellite = {**report_timelog,
                                      **report_attitude,
                                      **report_orbit,
                                      **report_subsystems,
                                      **report_ephemerides}
