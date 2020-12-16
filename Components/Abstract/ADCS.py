from .ComponentBase import ComponentBase
from Spacecraft.Components import Components
from Library.math_sup.Quaternion import Quaternions
from ..Logic.Control.Controller import Controller
from ..Logic.Estimation.error_state_kalman_filter.ESKF import ErrorStateKalmanFilter
from ..Logic.Estimation.error_state_kalman_filter.jacobians import Jacobians
from Library.math_sup.tools_reference_frame import unitVector
import numpy as np
import time
from Library.math_sup.tools_reference_frame import geodetic_to_ecef, gstime, rotationZ

REF_POINT = 2
NAD_POINT = 1
DETUMBLING = 0
GSPOINT = 3
# Target: Antenna Santiago
tar_alt = 572  # m
tar_long = -70.6506  # degree
tar_lat = -33.4372  # degree
twopi = np.pi * 2
deg2rad = np.pi / 180.0
rad2deg = 1 / deg2rad


class ADCS(ComponentBase):
    def __init__(self, init_componenets, subsystem_setting, dynamics):
        # prescalar_time in [ms]
        ComponentBase.__init__(self, prescalar_time=(1000 / subsystem_setting['ADCS_com_frequency']))
        # component quaternion
        self.q_b2c = subsystem_setting['q_b2c']
        # cycle in [ms]
        self.ctrl_cycle = 1000 / subsystem_setting['ADCS_com_frequency']
        self.port_id = subsystem_setting['port_id']
        self.comp_number = subsystem_setting['ADCS_COMPONENT_NUMBER']
        self.dynamics = dynamics
        self.components = Components(init_componenets, self.dynamics, self.port_id)
        self.current_omega_c_gyro = np.zeros(3)
        self.current_magVect_c_magSensor = np.zeros(3)
        self.magVect_i = np.zeros(3)
        self.magVect_b = np.zeros(3)
        self.sunPos_i = np.zeros(3)
        self.sunDir_b = np.zeros(3)
        self.sunPos_est_b = np.zeros(3)
        self.torque_rw_b = np.zeros(3)
        self.omega_b_est = np.zeros(3)
        self.b_dir = np.zeros(3)
        self.b_tar_b = np.zeros(3)
        self.control_torque = np.zeros(3)
        self.historical_control = []
        self.historical_estimation = []
        self.historical_P = []
        self.historical_P_det = []
        self.historical_P_det = []
        self.historical_magVect_i = []
        self.historical_ssEst_b = []
        self.historical_fssEst_b = []
        self.historical_fssQdr_d = []
        self.historical_ssDir_b = []
        self.historical_eskf_res = []
        self.historical_eskf_obserrmag = []
        self.historical_eskf_obserrcss = []
        self.historical_eskf_bias = []
        self.historical_theta_e = []
        self.historical_b_tar_b = []
        self.historical_b_dir_b = []
        self.historical_vec_dir_tar_b = []
        self.current_theta_e = 0
        self.vec_u_e = np.zeros(3)
        self.P_omega = subsystem_setting['P_omega']
        self.I_quat = subsystem_setting['I_quat']
        self.P_quat = subsystem_setting['P_quat']
        self.rw_torque_b = np.zeros(3)
        self.q_i2b_est = Quaternions([0, 0, 0, 1])
        self.q_i2b_est_eskf_temp = Quaternions([0, 0, 0, 1])
        self.q_b2b_now2tar = Quaternions([0, 0, 0, 1])
        self.q_i2b_tar = Quaternions([0, 0, 0, 1])
        self.eskf = ErrorStateKalmanFilter(dim_x=7, dim_dx=6, dim_u=3, dim_z=3, inertia=self.dynamics.attitude.Inertia,
                                           invInertia=self.dynamics.attitude.inv_Inertia)
        self.jacobians = Jacobians()
        self.number_ss = len(self.components.sunsensors)
        self.number_fss = len(self.components.fss)
        self.current_I_sunsensors = np.zeros(self.number_ss)
        self.V_ratios_fss = np.zeros((self.number_fss, 2))
        self.params_fss = [None] * self.number_fss
        self.rsfss_b = np.zeros((self.number_fss, 3))
        self.qdrsfss_b = np.zeros((self.number_fss, 2))
        self.tick_temp = 1

        self.adcs_mode = GSPOINT
        self.components.mtt.set_step_width(self.ctrl_cycle / 1000)
        for rw in self.components.rwmodel:
            rw.set_step_width(self.ctrl_cycle / 1000)

        # self.controller = Controller().pid(self.P_quat, self.I_quat, self.P_omega, self.ctrl_cycle/1000)
        control_parameters = {'N_pred_horiz': 10}
        # Geodetic to ECEF
        self.tar_pos_ecef = geodetic_to_ecef(tar_alt, tar_long * deg2rad, tar_lat * deg2rad)
        self.controller = Controller().mpc(self.dynamics, control_parameters, self.ctrl_cycle)

    def main_routine(self, count, sc_isDark):
        self.read_sensors(sc_isDark)

        self.check_mode()

        self.determine_attitude()

        self.calculate_control_torque()

        # self.calc_rw_torque()

        # self.calc_mtt_torque()
        return

    def read_sensors(self, sc_isDark):
        self.current_omega_c_gyro = self.components.gyro.measure(self.dynamics.attitude.current_omega_b)
        self.current_magVect_c_magSensor = self.components.mag.measure(self.magVect_b)
        for i in range(self.number_ss):
            self.components.sunsensors[i].measure(sc_isDark)
            self.current_I_sunsensors[i] = self.components.sunsensors[i].get_I_b()
        for i in range(self.number_fss):
            self.components.fss[i].measure(sc_isDark)
            self.V_ratios_fss[i] = self.components.fss[i].get_V_ratio_b()
            self.params_fss[i] = self.components.fss[i].calib_params

    def determine_attitude(self):
        # To do: Omega Estimate
        self.omega_b_est = self.current_omega_c_gyro
        # True/Simulated quaternion
        att = self.dynamics.attitude.get_current_q_i2b()
        self.q_i2b_est.setquaternion(att)
        # Simulated Sun Vector preprocessing
        self.sunPos_i = self.dynamics.ephemeris.selected_body['SUN'].get_pos_from_center_i()
        self.sunDir_b = unitVector(self.dynamics.ephemeris.selected_body['SUN'].get_pos_from_sc_b())
        # Coarse Sun Sensor preprocessing (vector extraction from currents)
        self.sunPos_est_b = self.eskf.get_ss_vect(self.current_I_sunsensors, self.components.sunsensors[0].I_max)
        # Fine Sun Sensor preprocessing (vector extraction from quadrature voltage)
        for i in range(self.number_fss):
            self.rsfss_b[i] = self.eskf.get_fss_vect(self.V_ratios_fss[i], self.params_fss[i])
            self.qdrsfss_b[i] = self.eskf.get_fss_qdvect(self.V_ratios_fss[i], self.params_fss[i])
        # Fine Sun Sensor average preprocessing (vector extraction from quadrature voltage)
        self.rsfss_mean_b = self.eskf.get_fss_mean_vect(self.rsfss_b)

        # Estimate Spacecraft Attitude quaternion
        # self.eskf_process(self.ctrl_cycle/1000)
        return

    def check_mode(self):
        if self.adcs_mode == DETUMBLING:
            self.omega_b_tar = np.array([0.0, 0.0, 0.0])
            # self.controller.set_gain(self.P_omega, self.I_quat, np.diag([0.0, 0.0, 0.0]))
        elif self.adcs_mode == NAD_POINT:
            print('Nadir pointing mode...')
        elif self.adcs_mode == REF_POINT:
            # Vector direction of the Body frame to point to another vector
            b_dir = np.array([0, 0, 1])

            # Vector target from Inertial frame
            i_tar = np.array([1, 1, 1])
            i_tar = i_tar / np.linalg.norm(i_tar)

            # Vector target from body frame
            b_tar = self.q_i2b_est.frame_conv(i_tar)
            b_tar /= np.linalg.norm(b_tar)

            b_lambda = np.cross(b_dir, b_tar)
            b_lambda /= np.linalg.norm(b_lambda)

            rot = np.arccos(np.dot(b_dir, b_tar))

            self.q_b2b_now2tar.setquaternion([b_lambda, rot])
            self.q_b2b_now2tar.normalize()
            self.q_i2b_tar = self.q_i2b_est * self.q_b2b_now2tar
        elif self.adcs_mode == GSPOINT:
            # Vector direction of the Body frame to point to another vector
            self.b_dir = np.array([0, 0, 1])
            # Error
            current_sideral = gstime(self.dynamics.simtime.current_jd)
            current_tar_pos_eci_earth = rotationZ(self.tar_pos_ecef, current_sideral)
            current_tar_s2tar_i = current_tar_pos_eci_earth - self.dynamics.orbit.current_position_i
            current_tar_pos_b = self.dynamics.attitude.current_quaternion_i2b.frame_conv(current_tar_s2tar_i)
            self.b_tar_b = current_tar_pos_b / np.linalg.norm(current_tar_pos_b)
            # self.current_theta_e = np.arccos(np.dot(self.b_dir, self.b_tar_b))
            self.current_theta_e = np.arccos(np.dot(self.b_dir, self.b_tar_b) / (np.linalg.norm(self.b_dir) * np.linalg.norm(self.b_tar_b)))
            self.vec_u_e = np.cross(self.b_dir, self.b_tar_b)
            self.vec_u_e /= np.linalg.norm(self.vec_u_e)
        else:
            print('No mode selected')

    def calculate_control_torque(self):
        q_b2i_est = Quaternions(self.q_i2b_est.conjugate())
        # First it is necessary to pass the quaternion from attitude to inertial,
        # then the target vector is rotated from the inertial to body frame
        q_i2b_now2tar = q_b2i_est * self.q_i2b_tar
        q_i2b_now2tar.normalize()

        torque_direction = np.zeros(3)
        torque_direction[0] = q_i2b_now2tar()[0]
        torque_direction[1] = q_i2b_now2tar()[1]
        torque_direction[2] = q_i2b_now2tar()[2]

        angle_rotation = 2 * np.arccos(q_i2b_now2tar()[3])

        # error_omega_ = self.omega_b_tar - self.omega_b_est
        # error_ = angle_rotation * torque_direction

        start_time = time.time()
        control_mag_torque = self.controller.open_loop([self.dynamics.attitude.current_quaternion_i2b,
                                                        self.dynamics.attitude.current_omega_b,
                                                        self.control_torque], self.dynamics.simtime.current_jd)

        print('Tiempo de calculo: ', time.time() - start_time)
        self.control_torque = control_mag_torque * self.vec_u_e

    def calc_mtt_torque(self):
        self.components.mtt.calc_torque(self.control_torque, self.current_magVect_c_magSensor)
        return

    def calc_rw_torque(self):
        f = 0
        for rw in self.components.rwmodel:
            rw.control_power()
            rw.set_torque(self.control_torque[f], self.ctrl_cycle)
            self.rw_torque_b += rw.calc_torque(self.ctrl_cycle)
            f += 1
        return

    def get_rwtorque(self):
        return self.rw_torque_b

    def set_magVector(self, mag_i, mag_b):
        self.magVect_i = mag_i
        self.magVect_b = mag_b

    def save_data(self):
        self.historical_control.append(self.control_torque)
        self.historical_magVect_i.append(self.magVect_i)
        self.historical_theta_e.append(self.current_theta_e)
        self.historical_b_tar_b.append(self.b_tar_b)
        self.historical_b_dir_b.append(self.b_dir)
        self.historical_vec_dir_tar_b.append(self.vec_u_e)

    def get_log_values(self, subsys):
        report = {'MTT_' + subsys + '_b(X)[Am]': 0,
                  'MTT_' + subsys + '_b(Y)[Am]': 0,
                  'MTT_' + subsys + '_b(Z)[Am]': 0}

        report_control = {'Control_' + subsys + '_b(X)[Nm]': np.array(self.historical_control)[:, 0],
                          'Control_' + subsys + '_b(Y)[Nm]': np.array(self.historical_control)[:, 1],
                          'Control_' + subsys + '_b(Z)[Nm]': np.array(self.historical_control)[:, 2]}
        report_target_state = {'Theta_error [rad]': np.array(self.historical_theta_e),
                               'Vector_dir_b(X) [-]': np.array(self.historical_b_dir_b)[:, 0],
                               'Vector_dir_b(Y) [-]': np.array(self.historical_b_dir_b)[:, 1],
                               'Vector_dir_b(Z) [-]': np.array(self.historical_b_dir_b)[:, 2],
                               'Vector_tar_b(X) [-]': np.array(self.historical_b_tar_b)[:, 0],
                               'Vector_tar_b(Y) [-]': np.array(self.historical_b_tar_b)[:, 1],
                               'Vector_tar_b(Z) [-]': np.array(self.historical_b_tar_b)[:, 2],
                               'Vector_dir_tar_b(X) [-]': np.array(self.historical_vec_dir_tar_b)[:, 0],
                               'Vector_dir_tar_b(Y) [-]': np.array(self.historical_vec_dir_tar_b)[:, 1],
                               'Vector_dir_tar_b(Z) [-]': np.array(self.historical_vec_dir_tar_b)[:, 2]}
        report = {**report, **report_control, **report_target_state}
        return report
