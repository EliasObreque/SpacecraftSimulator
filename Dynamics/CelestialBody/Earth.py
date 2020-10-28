
import numpy as np
from Library.math_sup.tools_reference_frame import gstime
twopi       = 2.0 * np.pi
deg2rad     = np.pi / 180.0


class Earth(object):
    def __init__(self, kernel_spk, wgs=2):
        self.kernel_spk = kernel_spk
        self.historical_gst = []
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

        self.current_sideral = 0
        self.load_properties(wgs)
        self.tolerance = 1e-10  # rad

    def save_data(self):
        self.historical_gst.append(self.current_sideral)
        self.historical_pos_from_center_i.append(self.current_pos_from_center_i)
        self.historical_vel_from_center_i.append(self.current_vel_from_center_i)
        self.historical_pos_from_sc_b.append(self.current_pos_from_sc_b)
        self.historical_vel_from_sc_b.append(self.current_vel_from_sc_b)
        self.historical_pos_from_sc_i.append(self.current_pos_from_sc_i)
        self.historical_vel_from_sc_i.append(self.current_vel_from_sc_i)

    def update_state(self, current_jd, center_object_pos_i, center_object_vel_i,
                     sc_pos_from_center_i, sc_vel_from_center_i, q_i2b, iscenter=False):
        emb_ssb_pos, emb_ssb_vel = self.kernel_spk[0, 3].compute_and_differentiate(current_jd)
        earth_emb_pos, earth_emb_vel = self.kernel_spk[3, 399].compute_and_differentiate(
            current_jd)  # vector from EMB 2 earth
        earth_ssb_pos = emb_ssb_pos + earth_emb_pos
        earth_ssb_vel = emb_ssb_vel + earth_emb_vel
        earth_ssb_pos *= 1000  # km to m
        earth_ssb_vel *= 1000 / 86400.0  # km/day to m/s
        if iscenter:
            return earth_ssb_pos, earth_ssb_vel
        self.current_pos_from_center_i = earth_ssb_pos - center_object_pos_i
        self.current_vel_from_center_i = earth_ssb_vel - center_object_vel_i
        self.current_pos_from_sc_i = self.current_pos_from_center_i - sc_pos_from_center_i
        self.current_vel_from_sc_i = self.current_vel_from_center_i - sc_vel_from_center_i
        self.current_pos_from_sc_b = q_i2b.frame_conv(self.current_pos_from_sc_i)
        self.current_vel_from_sc_b = q_i2b.frame_conv(self.current_vel_from_sc_i)
        self.current_sideral = gstime(current_jd)

    def get_current_sideral(self):
        return self.current_sideral

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

    def load_properties(self, wgs):
        if wgs == 0:
            #  ------------ wgs-72 old constants ------------
            self.radiuskm = 6378.135  # km
            self.mu = 3.9860079964e14  # in m3 / s2
            self.radiuskm = 6378.135  # km
            self.xke = 0.0743669161
            self.tumin = 1.0 / self.xke
            self.j2 = 0.001082616
            self.j3 = -0.00000253881
            self.j4 = -0.00000165597
            self.j3oj2 = self.j3 / self.j2
            self.f = 1.0 / 298.26
        elif wgs == 1:
            #  ------------ wgs-72 constants ------------
            self.radiuskm = 6378.135  # km
            self.mu = 3.986008e14  # in m3 / s2
            self.radiuskm = 6378.135  # km
            self.xke = 60.0 / np.sqrt(self.radiuskm * self.radiuskm * self.radiuskm / self.mu)
            self.tumin = 1.0 / self.xke
            self.j2 = 0.001082616
            self.j3 = -0.00000253881
            self.j4 = -0.00000165597
            self.j3oj2 = self.j3 / self.j2
            self.f = 1.0 / 298.26
        elif wgs == 2:
            #  ------------ wgs-84 constants ------------
            self.radiuskm = 6378.137  # km
            self.mu = 3.986005e14  # in m3 / s2
            self.xke = 60.0 / np.sqrt(self.radiuskm * self.radiuskm * self.radiuskm / self.mu)
            self.tumin = 1.0 / self.xke
            self.j2 = 0.00108262998905
            self.j3 = -0.00000253215306
            self.j4 = -0.00000161098761
            self.j3oj2 = self.j3 / self.j2
            self.f = 1.0 / 298.257223563
        else:
            print('wgs not used')
        self.e2 = self.f * (2 - self.f)

    def get_log_values(self):
        report = {'GST [rad]': self.historical_gst,
                  'Earth_pos_i(X) [m]': np.array(self.historical_pos_from_center_i)[:, 0],
                  'Earth_pos_i(Y) [m]': np.array(self.historical_pos_from_center_i)[:, 1],
                  'Earth_pos_i(Z) [m]': np.array(self.historical_pos_from_center_i)[:, 2],
                  'Earth_vel_i(X) [m]': np.array(self.historical_vel_from_center_i)[:, 0],
                  'Earth_vel_i(Y) [m]': np.array(self.historical_vel_from_center_i)[:, 1],
                  'Earth_vel_i(Z) [m]': np.array(self.historical_vel_from_center_i)[:, 2],
                  'Earth_pos_b(X) [m]': np.array(self.historical_pos_from_sc_b)[:, 0],
                  'Earth_pos_b(Y) [m]': np.array(self.historical_pos_from_sc_b)[:, 1],
                  'Earth_pos_b(Z) [m]': np.array(self.historical_pos_from_sc_b)[:, 2],
                  'Earth_vel_b(X) [m]': np.array(self.historical_vel_from_sc_b)[:, 0],
                  'Earth_vel_b(Y) [m]': np.array(self.historical_vel_from_sc_b)[:, 1],
                  'Earth_vel_b(Z) [m]': np.array(self.historical_vel_from_sc_b)[:, 2],
                  'Earth_pos_sc_i(X) [m]': np.array(self.historical_pos_from_sc_i)[:, 0],
                  'Earth_pos_sc_i(Y) [m]': np.array(self.historical_pos_from_sc_i)[:, 1],
                  'Earth_pos_sc_i(Z) [m]': np.array(self.historical_pos_from_sc_i)[:, 2],
                  'Earth_vel_sc_i(X) [m]': np.array(self.historical_vel_from_sc_i)[:, 0],
                  'Earth_vel_sc_i(Y) [m]': np.array(self.historical_vel_from_sc_i)[:, 1],
                  'Earth_vel_sc_i(Z) [m]': np.array(self.historical_vel_from_sc_i)[:, 2]}
        return report

