"""
Created by:

@authors: Elias Obreque, Gustavo Diaz
els.obrq@gmail.com


ref:
 - Orbital Mechanics for engineering students, Chapter 12, pag: 701-702
 - Attitude Determination – Advanced Sun Sensors for Pico-satellites, Yonatan Winetraub (a), San Bitan, Uval dd
Dr Anna B. Heller (b): https://pdfs.semanticscholar.org/4337/ba7deb320dd908a3e7f76b15d23d091d16e2.pdf
 - GomSpace NanoSense Fine Sun Sensor Manual
"""
import numpy as np
from Library.math_sup.Quaternion import Quaternions
from Library.math_sup.NormalRandom import NormalRandom
from ..Abstract.ComponentBase import ComponentBase
from Interface.SpacecraftInOut.SCIDriver import SCIDriver


class FineSunSensor(ComponentBase):
    def __init__(self, port_id, properties, dynamics):
        ComponentBase.__init__(self, 10)
        self.dynamics = dynamics
        self.port_id = port_id
        self.pos_vector = properties['pos_vector']
        self.normal_vector = properties['normal_vector']
        self.q_b2c = Quaternions(properties['q_b2c'])
        self.q_c2b = Quaternions(self.q_b2c.conjugate())
        self.calib_h = properties['h']
        self.calib_x0 = properties['x0']
        self.calib_y0 = properties['y0']
        self.calib_delta = np.deg2rad(properties['delta'])
        self.xyd_nr_std = properties['nr_stddev_c']
        self.scidriver = SCIDriver()
        self.xyd_nrs_c = NormalRandom([self.xyd_nr_std, 0, 0])
        self.scidriver.connect_port(self.port_id, 0, 0)
        self.current_sun_vector_b = np.zeros(3)
        self.current_sun_vector_c = np.zeros(3)
        self.historical_sun_vector_b = []
        self.historical_sun_vector_c = []
        self.historical_V_ratio_m = []
        self.historical_rd_m = []
        self.historical_theta_m = []
        self.historical_phi_m = []
        self.cos_theta = 0
        self.theta_fss = 0
        self.phi_fss = 0
        self.T = np.array([[np.cos(self.calib_delta), np.sin(self.calib_delta)], [-np.sin(self.calib_delta),
                                                                                  np.cos(self.calib_delta)]])
        self.Tinv = np.array([[np.cos(self.calib_delta), -np.sin(self.calib_delta)], [np.sin(self.calib_delta),
                                                                                      np.cos(self.calib_delta)]])
        self.calib_params = [self.calib_h, self.calib_x0, self.calib_y0, self.T, self.q_c2b]
        self.V0_ratio = np.array([self.calib_x0, self.calib_y0])
        self.V_ratio_m = np.zeros(2)
        self.rfss_c = np.zeros(3)
        self.rd_c = np.zeros(2)
        self.condition_ = False
        self.albeo_correction = False

    def main_routine(self, count, sc_isDark):
        self.measure(sc_isDark)

    def set_port_id(self, port):
        self.port_id = port

    def measure(self, sc_isDark):
        isDark = sc_isDark
        self.V_ratio_m = np.array([0, 0])
        if isDark:
            self.V_ratio_m = np.array([0, 0])
        else:
            self.calc_sun_vector_b()
        return

    def calc_sun_vector_b(self):
        sun_vector_from_c_b = self.dynamics.ephemeris.selected_body['SUN'].get_pos_from_sc_b() - self.pos_vector
        sun_unit_vector_from_c_b = sun_vector_from_c_b / np.linalg.norm(sun_vector_from_c_b)
        self.rfss_c = self.q_b2c.frame_conv(sun_unit_vector_from_c_b, "q")
        # Corroborates that the sensor reaches it light and is not obstructed by the satellite.
        self.calc_ang_parameters(sun_unit_vector_from_c_b, self.rfss_c)
        if self.condition_:
            if self.theta_fss < np.pi/3:    # Field of view (datasheet pag.8)
                xd = np.sign(self.rfss_c[2]) * self.calib_h * np.tan(self.theta_fss) * np.sin(self.phi_fss)
                yd = self.calib_h * np.tan(self.theta_fss) * np.cos(self.phi_fss)
                Vratio = self.Tinv.dot(np.array([xd, yd])) - self.V0_ratio
                self.V_ratio_m = Vratio + self.xyd_nrs_c()[0:2]
                self.rd_c = np.array([xd, yd])
            else:
                self.V_ratio_m = np.array([0, 0])

    def calc_ang_parameters(self, input_b_norm, rfss):
        self.cos_theta = np.inner(self.normal_vector, input_b_norm)
        self.condition_ = self.cos_theta > 0.0
        self.theta_fss = np.arccos(rfss[0])
        if self.theta_fss != 0:
            self.phi_fss = np.arccos(rfss[1]/np.sin(self.theta_fss))

    def get_vector_sun_b(self):
        return self.current_sun_vector_b

    def get_normal_vector_b(self):
        return self.normal_vector

    def get_V_ratio_b(self):
        return self.V_ratio_m

    def get_log_values(self, subsys):
        report = {'V_ratio_m': np.array(self.historical_V_ratio_m)}
        return report

    def log_value(self):
        self.historical_sun_vector_b.append(self.current_sun_vector_b)
        self.historical_V_ratio_m.append(self.V_ratio_m)
        self.historical_sun_vector_c.append(self.rfss_c)
        self.historical_rd_m.append(self.rd_c)
        self.historical_theta_m.append(self.theta_fss)
        self.historical_phi_m.append(self.phi_fss)
