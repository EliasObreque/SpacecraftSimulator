
import numpy as np
from Library.math_sup.Quaternion import Quaternions
from Library.math_sup.RandomWalk import RandomWalk
from Library.math_sup.NormalRandom import NormalRandom
from ..Abstract.ComponentBase import ComponentBase
from Interface.SpacecraftInOut.SCIDriver import SCIDriver


class Magnetometer(ComponentBase):
    def __init__(self, port_id, properties, dynamics):
        ComponentBase.__init__(self, 2)
        self.dynamics_in_mag = dynamics
        self.port_id = port_id
        self.current = properties['current']
        self.q_b2c = Quaternions(properties['q_b2c'])
        self.scalefactor = properties['ScaleFactor']
        self.bias_c = properties['Bias_c']
        self.nrs_c = NormalRandom(properties['nr_stddev_c'])
        self.n_rw_c = RandomWalk(properties['rw_stepwidth'], properties['rw_stddev_c'], properties['rw_limit_c'])
        self.current_magVect_c = np.zeros(3)
        self.historical_magVect_c = []
        self.scidriver = SCIDriver()
        self.scidriver.connect_port(self.port_id, 0, 0)

    def main_routine(self, count, sc_isDark):
        return

    def set_port_id(self, port):
        self.port_id = port

    def measure(self, magVect_b):
        # RangeCheck
        # TO DO

        # Convert magnetic vector from body (b) coordinate system to sensor actual coordinate system (c)
        magVect_c = self.q_b2c.frame_conv(magVect_b)

        # Addition of scale factor
        magVect_c = self.scalefactor.dot(magVect_c)

        # Addition of bias
        magVect_c += self.bias_c

        # Addition of random walk
        #magVect_c += self.n_rw_c()

        # Adding gaussian noise
        magVect_c += self.nrs_c()

        self.current_magVect_c = magVect_c
        return self.current_magVect_c

    def get_magVect_c(self):
        return self.current_magVect_c

    def get_log_values(self, subsys):
        report = {'magSensor_refVect' + subsys + '_c(X)[T?]': np.array(self.historical_magVect_c)[:, 0],
                  'magSensor_refVect' + subsys + '_c(Y)[T?]': np.array(self.historical_magVect_c)[:, 1],
                  'magSensor_refVect' + subsys + '_c(Z)[T?]': np.array(self.historical_magVect_c)[:, 2]}
        return report

    def get_current(self):
        return self.current

    def log_value(self):
        self.historical_magVect_c.append(self.current_magVect_c)
