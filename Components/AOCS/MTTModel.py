"""
Created: Elias Obreque- -2/26/2020

"""
from ..Abstract.ComponentBase import ComponentBase
import numpy as np
from Library.math_sup.Quaternion import Quaternions

RAD2RPM = 60 / (2 * np.pi)
RPM2RAD = 1 / RAD2RPM
# nano Tesla to Tesla
nT2T = 1e-9


class MTTModel(ComponentBase):
    def __init__(self, mtt_properties):
        ComponentBase.__init__(self, 50)
        self.q_b2c = Quaternions(mtt_properties['q_b2c'])
        self.max_c_am2 = mtt_properties['max_c_am2']
        self.min_c_am2 = mtt_properties['min_c_am2']
        self.bias_c = mtt_properties['bias_c']
        self.rw_stddev_c = mtt_properties['rw_stddev_c']
        self.rw_limit_c = mtt_properties['rw_limit_c']
        self.nr_stddev_c = mtt_properties['nr_stddev_c']
        self.step_width = mtt_properties['prop_step']
        self.historical_mtt_torque_b = []
        self.mtt_torque_b = np.zeros(3)

    def main_routine(self, count, sc_isDark):

        return

    def set_step_width(self, value):
        if value < self.step_width:
            self.step_width = value

    def calc_torque(self, control_torque_b, mag_earth_b):
        # Body frame to components frame
        mag_earth_c = self.q_b2c.frame_conv(mag_earth_b)

        control_mag_mom_b = np.cross(mag_earth_b, control_torque_b)
        control_mag_mom_c = self.q_b2c.frame_conv(control_mag_mom_b)

        for i in range(3):
            if control_mag_mom_c[i] > self.max_c_am2[i]:
                control_mag_mom_c[i] = self.max_c_am2[i]
            elif control_mag_mom_c[i] < self.min_c_am2[i]:
                control_mag_mom_c[i] = self.min_c_am2[i]

        mtt_torque_c = np.cross(control_mag_mom_c, nT2T * mag_earth_c)

        q_c2b = Quaternions(self.q_b2c.conjugate())
        self.mtt_torque_b = q_c2b.frame_conv(mtt_torque_c)
        return self.mtt_torque_b

    def get_torque(self):
        return self.mtt_torque_b

    def get_current(self):
        return

    def log_value(self):
        self.historical_mtt_torque_b.append(self.mtt_torque_b)

    def get_log_values(self, subsys):
        report = {'RWModel' + subsys + '_c(X)[Nm]': np.array(self.historical_mtt_torque_b)[:, 0],
                  'RWModel' + subsys + '_c(Y)[Nm]': np.array(self.historical_mtt_torque_b)[:, 1],
                  'RWModel' + subsys + '_c(Z)[Nm]': np.array(self.historical_mtt_torque_b)[:, 2]}
        return report
