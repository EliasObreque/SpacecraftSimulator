
import numpy as np
from Library.math_sup.tools_reference_frame import gstime
twopi       = 2.0 * np.pi
deg2rad     = np.pi / 180.0


class Sun(object):
    def __init__(self, kernel_spk):
        self.kernel_spk = kernel_spk
        self.mu = 1.32712e+20
        self.current_pos_from_center_i = np.zeros(3)
        self.current_vel_from_center_i = np.zeros(3)
        self.current_pos_from_sc_b = np.zeros(3)
        self.current_vel_from_sc_b = np.zeros(3)
        self.current_pos_from_sc_i = np.zeros(3)
        self.current_vel_from_sc_i = np.zeros(3)
        self.historical_pos_from_center_i = []
        self.historical_vel_from_center_i = []
        self.historical_pos_from_sc_b = []
        self.historical_vel_from_sc_b = []
        self.historical_pos_from_sc_i = []
        self.historical_vel_from_sc_i = []

    def save_data(self):
        self.historical_pos_from_center_i.append(self.current_pos_from_center_i)
        self.historical_vel_from_center_i.append(self.current_vel_from_center_i)
        self.historical_pos_from_sc_b.append(self.current_pos_from_sc_b)
        self.historical_vel_from_sc_b.append(self.current_vel_from_sc_b)
        self.historical_pos_from_sc_i.append(self.current_pos_from_sc_i)
        self.historical_vel_from_sc_i.append(self.current_vel_from_sc_i)

    def update_state(self, current_jd, center_object_pos_i, center_object_vel_i,
                     sc_pos_from_center_i, sc_vel_from_center_i, q_i2b, iscenter=False):
        sun_ssb_pos, sun_ssb_vel = self.kernel_spk[0, 10].compute_and_differentiate(current_jd)  # vector from SSB 2 sun
        if iscenter:
            return sun_ssb_pos, sun_ssb_vel
        self.current_pos_from_center_i = sun_ssb_pos * 1000 - center_object_pos_i   # km to m
        self.current_vel_from_center_i = sun_ssb_vel * 1000 / 86400.0 - center_object_vel_i # km/day to m/s
        self.current_pos_from_sc_i = self.current_pos_from_center_i - sc_pos_from_center_i
        self.current_vel_from_sc_i = self.current_vel_from_center_i - sc_vel_from_center_i
        self.current_pos_from_sc_b = q_i2b.frame_conv(self.current_pos_from_sc_i)
        self.current_vel_from_sc_b = q_i2b.frame_conv(self.current_vel_from_sc_i)

    def get_pos_from_center_i(self):
        return self.current_pos_from_center_i

    def get_vel_from_center_i(self):
        return self.current_vel_from_center_i

    def get_pos_from_sc_i(self):
        return self.current_pos_from_sc_i

    def get_vel_from_sc_i(self):
        return self.current_vel_from_sc_i

    def get_pos_from_sc_b(self):
        return self.current_pos_from_sc_b

    def get_vel_from_sc_b(self):
        return self.current_vel_from_sc_b

    def get_log_values(self):
        report = {'Sun_pos_i(X) [m]': np.array(self.historical_pos_from_center_i)[:, 0],
                  'Sun_pos_i(Y) [m]': np.array(self.historical_pos_from_center_i)[:, 1],
                  'Sun_pos_i(Z) [m]': np.array(self.historical_pos_from_center_i)[:, 2],
                  'Sun_vel_i(X) [m]': np.array(self.historical_vel_from_center_i)[:, 0],
                  'Sun_vel_i(Y) [m]': np.array(self.historical_vel_from_center_i)[:, 1],
                  'Sun_vel_i(Z) [m]': np.array(self.historical_vel_from_center_i)[:, 2],
                  'Sun_pos_b(X) [m]': np.array(self.historical_pos_from_sc_b)[:, 0],
                  'Sun_pos_b(Y) [m]': np.array(self.historical_pos_from_sc_b)[:, 1],
                  'Sun_pos_b(Z) [m]': np.array(self.historical_pos_from_sc_b)[:, 2],
                  'Sun_vel_b(X) [m]': np.array(self.historical_vel_from_sc_b)[:, 0],
                  'Sun_vel_b(Y) [m]': np.array(self.historical_vel_from_sc_b)[:, 1],
                  'Sun_vel_b(Z) [m]': np.array(self.historical_vel_from_sc_b)[:, 2],
                  'Sun_pos_sc_i(X) [m]': np.array(self.historical_pos_from_sc_i)[:, 0],
                  'Sun_pos_sc_i(Y) [m]': np.array(self.historical_pos_from_sc_i)[:, 1],
                  'Sun_pos_sc_i(Z) [m]': np.array(self.historical_pos_from_sc_i)[:, 2],
                  'Sun_vel_sc_i(X) [m]': np.array(self.historical_vel_from_sc_i)[:, 0],
                  'Sun_vel_sc_i(Y) [m]': np.array(self.historical_vel_from_sc_i)[:, 1],
                  'Sun_vel_sc_i(Z) [m]': np.array(self.historical_vel_from_sc_i)[:, 2]}
        return report
