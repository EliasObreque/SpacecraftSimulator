
"""
Created by:

@author: Elias Obreque
els.obrq@gmail.com


ref:
 - Orbital Mechanics for engineering students, Chapter 12, pag: 701-702
 - Attitude Determination – Advanced Sun Sensors for Pico-satellites, Yonatan Winetraub (a), San Bitan, Uval dd
Dr Anna B. Heller (b): https://pdfs.semanticscholar.org/4337/ba7deb320dd908a3e7f76b15d23d091d16e2.pdf
"""
import numpy as np
from Library.math_sup.NormalRandom import NormalRandom
from ..Abstract.ComponentBase import ComponentBase
from Interface.SpacecraftInOut.SCIDriver import SCIDriver


class SunSensor(ComponentBase):
    def __init__(self, port_id, properties, dynamics):
        ComponentBase.__init__(self, 1)
        self.dynamics = dynamics
        self.port_id = port_id
        self.pos_vector = properties['pos_vector']
        self.normal_vector = properties['normal_vector']
        self.cosine_error = np.deg2rad(properties['cosine_error'])
        self.I_nr_std = properties['nr_stddev_c']
        self.I_max = properties['I_max']
        self.scidriver = SCIDriver()
        self.nrs_c = NormalRandom([self.cosine_error, 0, 0])
        self.nrs_I = NormalRandom([self.I_nr_std, 0, 0])
        self.scidriver.connect_port(self.port_id, 0, 0)
        self.current_sun_vector_b = np.zeros(3)
        self.current_sun_vector_c = np.zeros(3)
        self.historical_sun_vector_b = []
        self.historical_I_measured = []
        self.I_measured = 0
        self.condition_ = False
        self.albeo_correction = False
        self.isDark = False

    def main_routine(self, count, sc_isDark):
        self.measure(sc_isDark)

    def set_port_id(self, port):
        self.port_id = port

    def measure(self, sc_isDark):
        isDark = sc_isDark
        self.I_measured = 0
        if isDark:
            self.I_measured = 0
        else:
            self.calc_sun_vector_b()
        return

    def calc_sun_vector_b(self):
        sun_vector_from_c_b = self.dynamics.ephemeris.selected_body['SUN'].get_pos_from_sc_b() - self.pos_vector
        sun_unit_vector_from_c_b = sun_vector_from_c_b / np.linalg.norm(sun_vector_from_c_b)
        # Corroborates that the sensor reaches it light and is not obstructed by the satellite.
        self.calc_ang_parameters(sun_unit_vector_from_c_b)
        if self.condition_:
            theta = np.arccos(self.cos_theta) + self.nrs_c()[0]
            # El valor absoluto evita que el ruido gaussiano de un coseno de theta bajo cero cuando el coseno de theta
            # original es cercano o igual a cero.
            cos_theta = np.abs(np.cos(theta))
            self.I_measured = self.I_max * cos_theta + self.nrs_I()[0]
            if self.I_measured < 0:
                self.I_measured = 0

    def calc_ang_parameters(self, input_b_norm):
        self.cos_theta = np.inner(self.normal_vector, input_b_norm)
        self.condition_ = self.cos_theta > 0.0

    def get_vector_sun_b(self):
        return self.current_sun_vector_b

    def get_I_normal_vector_b(self):
        return [self.I_measured, self.normal_vector]

    def get_I_b(self):
        return self.I_measured

    def get_log_values(self, subsys):
        report = {'I_measured': np.array(self.historical_I_measured)}
        return report

    def log_value(self):
        self.historical_sun_vector_b.append(self.current_sun_vector_b)
        self.historical_I_measured.append(self.I_measured)
