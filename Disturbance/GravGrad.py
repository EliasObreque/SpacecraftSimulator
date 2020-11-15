"""

ref:
- Yang, Yaguang - Spacecraft modeling, attitude determination, and control_ quaternion-based approach-CRC Press (2019)
    Chapter: 5.1
els.obrq@gmail.com
"""


import numpy as np


class GravGrad(object):
    def __init__(self, grav_dist_properties, spacecraft):
        self.dist_flag      = grav_dist_properties['gra_calculation']
        self.grav_logging   = grav_dist_properties['gra_logging']
        self.mu_planet      = spacecraft.dynamics.ephemeris.center_object_mu
        self.current_grav_torque_b = np.zeros(3)

    def get_torque_b(self):
        return self.current_grav_torque_b

    def update(self, environment, spacecraft):
        temp = spacecraft.dynamics.ephemeris
        self.calc_torque_b(temp.selected_body[temp.center_object].current_pos_from_sc_b,
                           spacecraft.dynamics.attitude.mpc_inertia)

    def calc_torque_b(self, r_b, Inertia_b):
        norm_r_b = np.linalg.norm(r_b)
        self.current_grav_torque_b = 3 * (self.mu_planet / norm_r_b ** 5) * (np.cross(r_b, Inertia_b.dot(r_b)))

    def get_force_b(self):
        return np.zeros(3)

