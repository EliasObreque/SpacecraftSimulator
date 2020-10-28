from .ComponentBase import ComponentBase
from Spacecraft.Components import Components
import numpy as np


class POWER(ComponentBase):
    def __init__(self, init_componenets, dynamics):
        ComponentBase.__init__(self, 5)
        self.dynamics = dynamics
        self.components = Components(init_componenets, self.dynamics, 3)
        self.current_voltage = 0

    def main_routine(self, count, sc_isDark):
        return

    def check_mode(self):
        return

    def get_log_values(self, subsys):
        report = {}
        if hasattr(self.components, 'gyro'):
            gyro = self.components.gyro
            report['gyro_omega_' + subsys + '_c(X)[rad/s]'] = np.array(gyro.historical_omega_c)[:, 0]
            report['gyro_omega_' + subsys + '_c(Y)[rad/s]'] = np.array(gyro.historical_omega_c)[:, 1]
            report['gyro_omega_' + subsys + '_c(Z)[rad/s]'] = np.array(gyro.historical_omega_c)[:, 2]
        return report
