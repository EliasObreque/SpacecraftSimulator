from .ComponentBase import ComponentBase
from Spacecraft.Components import Components
import numpy as np


class ODCS(ComponentBase):
    def __init__(self, init_componenets, dynamics):
        ComponentBase.__init__(self, 5)
        self.dynamics = dynamics
        self.components = Components(init_componenets, self.dynamics, 3)
        self.current_omega_c_gyro = np.zeros(3)
        self.force_thruster_b = np.zeros(3)
        self.torque_thruster_b = np.zeros(3)

    def main_routine(self, count, sc_isDark):

        self.read_sensors()

        self.determine_attitude()

        self.determine_orbit()

        self.check_mode()

        self.calculate_control_force()

        self.calc_thruster_force()
        return

    def read_sensors(self):
        self.current_omega_c_gyro = self.components.gyro.measure(self.dynamics.attitude.current_omega_b)

    def determine_attitude(self):
        return

    def determine_orbit(self):
        return

    def check_mode(self):
        return

    def calculate_control_force(self):
        return

    def calc_thruster_force(self):
        return

    def get_force(self):
        return self.force_thruster_b

    def get_torque(self):
        return self.torque_thruster_b

    def get_log_values(self, subsys):
        report = {}
        if hasattr(self.components, 'gyro'):
            gyro = self.components.gyro
            report['gyro_omega_' + subsys + '_c(X)[rad/s]'] = np.array(gyro.historical_omega_c)[:, 0]
            report['gyro_omega_' + subsys + '_c(Y)[rad/s]'] = np.array(gyro.historical_omega_c)[:, 1]
            report['gyro_omega_' + subsys + '_c(Z)[rad/s]'] = np.array(gyro.historical_omega_c)[:, 2]
        return report
