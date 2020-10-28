
import numpy as np
from Library.math_sup.Quaternion import Quaternions
from Components.Abstract.ComponentBase import ComponentBase
from Interface.SpacecraftInOut.SCIDriver import SCIDriver


class GroundStation(ComponentBase):
    def __init__(self, properties, dynamics):
        ComponentBase.__init__(self, 2)
        self.dynamics_in_mag = dynamics
        self.select_gs = {}
        print("-----------------------------")
        print('Ground Station:')
        for i in range(len(properties)):
            self.select_gs[properties[i]['gs_name']] = {'alt': properties[i]['alt'],
                                                        'long': properties[i]['long'],
                                                        'lat': properties[i]['lat']}
            print(' - ' + properties[i]['gs_name'] + ': ' + 'Altitude: ' + str(properties[i]['alt']) +
                  ' [m] - Longitude: ' + str(properties[i]['long']) + ' [°] - Latitude: ' + str(properties[i]['lat']) + ' [°]')

    def main_routine(self, count, sc_isDark):
        return

    def measure(self, magVect_b):
        # RangeCheck
        # TO DO
        return 0

    def get_magVect_c(self):
        return 0

    def get_log_values(self, subsys):
        return 0

    def get_current(self):
        return 0

    def log_value(self):
        0
