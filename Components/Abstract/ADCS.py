from .ComponentBase import ComponentBase
from Spacecraft.Components import Components
from Library.math_sup.Quaternion import Quaternions
from ..Logic.Control.Controller import Controller
from ..Logic.Estimation.error_state_kalman_filter.ESKF import ErrorStateKalmanFilter
from ..Logic.Estimation.error_state_kalman_filter.jacobians import Jacobians
from Library.math_sup.tools_reference_frame import unitVector
import numpy as np

REF_POINT = 2
NAD_POINT = 1
DETUMBLING = 0


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
        self.P_omega = subsystem_setting['P_omega']
        self.I_quat = subsystem_setting['I_quat']
        self.P_quat = subsystem_setting['P_quat']
        self.rw_torque_b = np.zeros(3)
        self.q_i2b_est = Quaternions([0, 0, 0, 1])
        self.q_i2b_est_eskf_temp = Quaternions([0, 0, 0, 1])
        self.q_b2b_now2tar = Quaternions([0, 0, 0, 1])
        self.q_i2b_tar = Quaternions([0, 0, 0, 1])
        self.eskf = ErrorStateKalmanFilter(dim_x=7, dim_dx=6, dim_u=3, dim_z=3, inertia=self.dynamics.attitude.Inertia, invInertia=self.dynamics.attitude.inv_Inertia)
        self.jacobians = Jacobians()
        self.number_ss = len(self.components.sunsensors)
        self.number_fss = len(self.components.fss)
        self.current_I_sunsensors = np.zeros(self.number_ss)
        self.V_ratios_fss = np.zeros((self.number_fss, 2))
        self.params_fss = [None]*self.number_fss
        self.rsfss_b = np.zeros((self.number_fss, 3))
        self.qdrsfss_b = np.zeros((self.number_fss, 2))
        self.tick_temp = 1

        self.adcs_mode = DETUMBLING
        for rw in self.components.rwmodel:
            rw.set_step_width(self.ctrl_cycle / 1000)
        self.controller = Controller().pid(self.P_quat, self.I_quat, self.P_omega, self.ctrl_cycle/1000)

    def main_routine(self, count, sc_isDark):
        self.read_sensors(sc_isDark)

        self.check_mode()

        self.determine_attitude()

        self.calculate_control_torque()

        self.calc_rw_torque()
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
        self.eskf_process(self.ctrl_cycle/1000)
        return

    def eskf_process(self, dt):#, gyroData, magData, dt, i):
        """
        * Iteration of the Fusion algorithm consist of predict of 
        * the nominal state, prediction of the error state, update
        * of the error estate and injection of error into nominal states
        *
        * @param gyroData np.array((3,1)) Gyroscope (x,y,z) noisy measurements
        * @param magData np.array((3,1)) Magnetometer (x,y,z) noisy measurements
        * @param dt Float Time step at which the algorithm runs
        * @param i Int Current iteration of the algorithm
        * @return self.eskf.q np.array((4,1)) Estimated quaternion
        """

        # Pridict state using gyro measurments
        u = self.current_omega_c_gyro
        self.eskf.predict_nominal(u, dt)
        self.eskf.predict_error(u, dt)
        # Update state at magnetometer data rate
        if self.tick_temp%(self.components.gyro.prescalar/self.components.mag.prescalar) == 0:
            z = unitVector(self.current_magVect_c_magSensor)
            self.eskf.update(z, self.jacobians.Hx_mag, self.jacobians.hx_mag, self.magVect_i, "mag")
            # self.eskf.reset()
        # Update state at coarse sun sensor data rate
        if self.tick_temp%(self.components.gyro.prescalar/self.components.sunsensors[0].prescalar) == 0:
            z = unitVector(self.sunPos_est_b)
            if np.linalg.norm(z)>0:
                self.eskf.update(z, self.jacobians.Hx_sun, self.jacobians.hx_sun, self.sunPos_i, "css")
                # self.eskf.reset()
        # Update state at fine sun sensor data rate
        # for i in range(self.number_fss):
        #     if self.tick_temp%(self.components.gyro.prescalar/self.components.fss[i].prescalar) == 0:
        #         z = unitVector(self.rsfss_b[i])
        #         if np.linalg.norm(z)>0:
        #             self.eskf.update(z, self.jacobians.Hx_sun, self.jacobians.hx_sun, self.sunPos_i, "fss")
        #             # self.eskf.reset()
        # Update state at mean fine sun sensor data rate
        if self.tick_temp%(self.components.gyro.prescalar/self.components.fss[0].prescalar) == 0:
                z = unitVector(self.rsfss_mean_b)
                if np.linalg.norm(z)>0:
                    self.eskf.update(z, self.jacobians.Hx_sun, self.jacobians.hx_sun, self.sunPos_i, "fss")
                    # self.eskf.reset()

        self.q_i2b_est_eskf_temp = self.eskf.q
        self.tick_temp+=1

    def check_mode(self):
        if self.adcs_mode == DETUMBLING:
            self.omega_b_tar = np.array([0.0, 0.0, 0.0])
            self.controller.set_gain(self.P_omega, self.I_quat, np.diag([0.0, 0.0, 0.0]))
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

        error_omega_ = self.omega_b_tar - self.omega_b_est
        error_ = angle_rotation * torque_direction
        control = self.controller.calc_control(error_, error_omega_, self.adcs_mode)
        self.control_torque = control

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
        self.historical_estimation.append(self.q_i2b_est_eskf_temp())
        self.historical_P.append(self.eskf.P)
        self.historical_P_det.append(np.linalg.det(self.eskf.P))
        self.historical_magVect_i.append(self.magVect_i)
        self.historical_eskf_res.append(self.eskf.y)
        self.historical_ssEst_b.append(unitVector(self.sunPos_est_b))
        self.historical_ssDir_b.append(self.sunDir_b)
        self.historical_eskf_obserrmag.append(self.eskf.dtheta_mag)
        self.historical_eskf_obserrcss.append(self.eskf.dtheta_css)
        self.historical_fssEst_b.append(np.array(self.rsfss_b))
        self.historical_fssQdr_d.append(np.array(self.qdrsfss_b))
        self.historical_eskf_bias.append(self.eskf.wb)

    def get_log_values(self, subsys):
        report = {'RWModel_' + subsys + '_b(X)[Nm]': 0,
                  'RWModel_' + subsys + '_b(Y)[Nm]': 0,
                  'RWModel_' + subsys + '_b(Z)[Nm]': 0}
        if hasattr(self.components, 'gyro'):
            gyro = self.components.gyro
            report['gyro_omega_' + subsys + '_c(X)[rad/s]'] = np.array(gyro.historical_omega_c)[:, 0]
            report['gyro_omega_' + subsys + '_c(Y)[rad/s]'] = np.array(gyro.historical_omega_c)[:, 1]
            report['gyro_omega_' + subsys + '_c(Z)[rad/s]'] = np.array(gyro.historical_omega_c)[:, 2]
            report['gyro_rw_' + subsys + '_c(X)[rad/s]'] = np.array(gyro.historical_rw_c_tmp)[:, 0]
            report['gyro_rw_' + subsys + '_c(Y)[rad/s]'] = np.array(gyro.historical_rw_c_tmp)[:, 1]
            report['gyro_rw_' + subsys + '_c(Z)[rad/s]'] = np.array(gyro.historical_rw_c_tmp)[:, 2]
            report['gyro_nr_' + subsys + '_c(X)[rad/s]'] = np.array(gyro.historical_nr_c_tmp)[:, 0]
            report['gyro_nr_' + subsys + '_c(Y)[rad/s]'] = np.array(gyro.historical_nr_c_tmp)[:, 1]
            report['gyro_nr_' + subsys + '_c(Z)[rad/s]'] = np.array(gyro.historical_nr_c_tmp)[:, 2]
        if hasattr(self.components, 'mag'):
            mag = self.components.mag
            report['magnetometer_vect_' + subsys + '_c(X)[T?]'] = np.array(mag.historical_magVect_c)[:, 0]
            report['magnetometer_vect_' + subsys + '_c(Y)[T?]'] = np.array(mag.historical_magVect_c)[:, 1]
            report['magnetometer_vect_' + subsys + '_c(Z)[T?]'] = np.array(mag.historical_magVect_c)[:, 2]
            report['magnetometer_vect_' + subsys + '_i(X)[T?]'] = np.array(self.historical_magVect_i)[:, 0]
            report['magnetometer_vect_' + subsys + '_i(Y)[T?]'] = np.array(self.historical_magVect_i)[:, 1]
            report['magnetometer_vect_' + subsys + '_i(Z)[T?]'] = np.array(self.historical_magVect_i)[:, 2]
        if hasattr(self.components, 'rwmodel'):
            for rw in self.components.rwmodel:
                report['RWModel_' + subsys + '_b(X)[Nm]'] += np.array(rw.historical_rw_torque_b)[:, 0]
                report['RWModel_' + subsys + '_b(Y)[Nm]'] += np.array(rw.historical_rw_torque_b)[:, 1]
                report['RWModel_' + subsys + '_b(Z)[Nm]'] += np.array(rw.historical_rw_torque_b)[:, 2]
        if hasattr(self.components, 'sunsensors'):
            ss_count = 1
            for ss in self.components.sunsensors:
                report['SS_(' + str(ss_count) + ')_I' + subsys + '_[uA]'] = np.array(ss.historical_I_measured)
                ss_count += 1
            report['SS_est_'+subsys+'_b(X)[unit]'] = np.array(self.historical_ssEst_b)[:, 0]
            report['SS_est_'+subsys+'_b(Y)[unit]'] = np.array(self.historical_ssEst_b)[:, 1]
            report['SS_est_'+subsys+'_b(Z)[unit]'] = np.array(self.historical_ssEst_b)[:, 2]
            report['SS_dir_'+subsys+'_b(X)[unit]'] = np.array(self.historical_ssDir_b)[:, 0]
            report['SS_dir_'+subsys+'_b(Y)[unit]'] = np.array(self.historical_ssDir_b)[:, 1]
            report['SS_dir_'+subsys+'_b(Z)[unit]'] = np.array(self.historical_ssDir_b)[:, 2]
        if hasattr(self.components, 'fss'):
            fss_count = 1
            for fss in self.components.fss:
                # Voltage Ratios (Vr)
                report['FSS_(' + str(fss_count) + ')_Vrx_' + subsys + '_[-]'] = np.array(fss.historical_V_ratio_m)[:, 0]
                report['FSS_(' + str(fss_count) + ')_Vry_' + subsys + '_[-]'] = np.array(fss.historical_V_ratio_m)[:, 1]

                report['FSS_(' + str(fss_count) + ')_xd_m_' + subsys + '_[-]'] = np.array(fss.historical_rd_m)[:, 0]
                report['FSS_(' + str(fss_count) + ')_yd_m_' + subsys + '_[-]'] = np.array(fss.historical_rd_m)[:, 1]

                report['FSS_(' + str(fss_count) + ')_rx_m_' + subsys + '_[-]'] = np.array(fss.historical_sun_vector_c)[:, 0]
                report['FSS_(' + str(fss_count) + ')_ry_m_' + subsys + '_[-]'] = np.array(fss.historical_sun_vector_c)[:, 1]
                report['FSS_(' + str(fss_count) + ')_rz_m_' + subsys + '_[-]'] = np.array(fss.historical_sun_vector_c)[:, 2]

                report['FSS_(' + str(fss_count) + ')_theta_m_' + subsys + '_[rad]'] = np.array(fss.historical_theta_m)
                report['FSS_(' + str(fss_count) + ')_phi_m_' + subsys + '_[rad]'] = np.array(fss.historical_phi_m)

                # Sun unit vector extraction (r)
                report['FSS_(' + str(fss_count) + ')_rx_' + subsys + '_[-]'] = np.array(self.historical_fssEst_b)[:, fss_count-1, 0]
                report['FSS_(' + str(fss_count) + ')_ry_' + subsys + '_[-]'] = np.array(self.historical_fssEst_b)[:, fss_count-1, 1]
                report['FSS_(' + str(fss_count) + ')_rz_' + subsys + '_[-]'] = np.array(self.historical_fssEst_b)[:, fss_count-1, 2]
                # Sun quadrature d-frame vector 
                # Sun unit vector extraction (r)
                report['FSS_(' + str(fss_count) + ')_xd_' + subsys + '_[-]'] = np.array(self.historical_fssQdr_d)[:, fss_count-1, 0]
                report['FSS_(' + str(fss_count) + ')_yd_' + subsys + '_[-]'] = np.array(self.historical_fssQdr_d)[:, fss_count-1, 1]
                fss_count += 1

        report_control = {'Control_' + subsys + '_b(X)[Nm]': np.array(self.historical_control)[:, 0],
                          'Control_' + subsys + '_b(Y)[Nm]': np.array(self.historical_control)[:, 1],
                          'Control_' + subsys + '_b(Z)[Nm]': np.array(self.historical_control)[:, 2]}
        report_estimation = {subsys +'_q_estEskfTemp_i2b(0)[-]': np.array(self.historical_estimation)[:, 0],
                          subsys +'_q_estEskfTemp_i2b(1)[-]': np.array(self.historical_estimation)[:, 1],
                          subsys +'_q_estEskfTemp_i2b(2)[-]': np.array(self.historical_estimation)[:, 2],
                          subsys +'_q_estEskfTemp_i2b(3)[-]': np.array(self.historical_estimation)[:, 3]}

        report_estimation2 = {subsys +'_P_est(0,0)[-]': np.array(self.historical_P)[:, 0, 0],
                          subsys +'_P_est(1,1)[-]': np.array(self.historical_P)[:, 1, 1],
                          subsys +'_P_est(2,2)[-]': np.array(self.historical_P)[:, 2, 2],
                          subsys +'_P_est_det[-]': np.array(self.historical_P_det)}

        report_estimation3 = {subsys +'_eskfRes(X)[-]': np.array(self.historical_eskf_res)[:, 0],
                          subsys +'_eskfRes(Y)[-]': np.array(self.historical_eskf_res)[:, 1],
                          subsys +'_eskfRes(Z)[-]': np.array(self.historical_eskf_res)[:, 2]}

        report_estimation4 = {subsys +'_eskfbias(X)[rad/s]': np.array(self.historical_eskf_bias)[:, 0],
                          subsys +'_eskfbias(Y)[rad/s]': np.array(self.historical_eskf_bias)[:, 1],
                          subsys +'_eskfbias(Z)[rad/s]': np.array(self.historical_eskf_bias)[:, 2]}

        report_estimation3[subsys +'_eskfobserrmag(X)[rad]'] = np.array(self.historical_eskf_obserrmag)[:, 0]
        report_estimation3[subsys +'_eskfobserrmag(Y)[rad]'] = np.array(self.historical_eskf_obserrmag)[:, 1]
        report_estimation3[subsys +'_eskfobserrmag(Z)[rad]'] = np.array(self.historical_eskf_obserrmag)[:, 2]

        report_estimation3[subsys +'_eskfobserrcss(X)[rad]'] = np.array(self.historical_eskf_obserrcss)[:, 0]
        report_estimation3[subsys +'_eskfobserrcss(Y)[rad]'] = np.array(self.historical_eskf_obserrcss)[:, 1]
        report_estimation3[subsys +'_eskfobserrcss(Z)[rad]'] = np.array(self.historical_eskf_obserrcss)[:, 2]

        report = {**report, **report_control, **report_estimation, **report_estimation2, **report_estimation3, **report_estimation4}
        return report
