"""
Created by:

@author: Elias Obreque
@Date: 11/13/2020 9:25 PM 
els.obrq@gmail.com

"""
import numpy as np
from copy import deepcopy, copy
from Library.math_sup.tools_reference_frame import JdToDecyear
from Library.igrf.IGRF import calculate_igrf
from Library.math_sup.Quaternion import Quaternions
from Library.math_sup.tools_reference_frame import gstime
from Library.math_sup.tools_reference_frame import fmod2
from Library.math_sup.tools_reference_frame import rotationY, rotationZ
inv_sec_day = 1 / (60.0 * 60.0 * 24.0)
twopi = np.pi * 2
#  ------------ wgs-84 constants ------------
radius_earth = 6378.137  # km
earth_flat = 1.0 / 298.257223563
earth_e2 = earth_flat * (2 - earth_flat)
geod_tolerance = 1e-10  # rad


class MPC(object):
    def __init__(self, dynamics, controller_parameters, ctrl_cycle):
        self.mpc_inertia = dynamics.attitude.Inertia
        self.mpc_inv_inertia = dynamics.attitude.inv_Inertia
        self.mpc_current_omega_b = np.zeros(3)
        self.mpc_current_quaternion_i2b = np.zeros(4)
        self.mpc_current_h_total_b = np.zeros(3)
        self.mpc_current_torque_b = np.zeros(3)

        self.mpc_dynamics_orb = deepcopy(dynamics.orbit.propagator_model)
        self.N_pred_horiz = controller_parameters['N_pred_horiz']
        self.mpc_main_count_time = 0
        self.mpc_start_jd = dynamics.simtime.current_jd
        self.mpc_future_jd = self.mpc_start_jd
        self.mpc_att_step_prop = dynamics.simtime.attitudestep
        self.mpc_orb_step_prop = dynamics.simtime.orbitstep
        self.mpc_sim_step_prop = dynamics.simtime.stepsimTime
        self.mpc_ctrl_cycle = ctrl_cycle

    def open_loop(self, current_att_state, current_jd):
        # Current state
        self.mpc_current_quaternion_i2b = current_att_state[0]
        self.mpc_current_omega_b = current_att_state[1]
        self.mpc_current_torque_b = current_att_state[2]

        self.mpc_start_jd = current_jd
        self.mpc_main_count_time = 0

        #self.mpc_current_h_total_b, h_total_i = self.calangmom(self.mpc_current_omega_b,
        #                                                       self.mpc_current_quaternion_i2b)

        last_omega_b = self.mpc_current_omega_b
        last_quaternion_q_i2b = self.mpc_current_quaternion_i2b
        last_torque_b = self.mpc_current_torque_b

        last_pos_i, last_vel_i = self.mpc_dynamics_orb.update_state_orbit(current_jd)
        last_sideral = gstime(current_jd)

        for i in range(self.N_pred_horiz):
            self.mpc_main_count_time += self.mpc_sim_step_prop

            self.mpc_future_jd = self.mpc_start_jd + self.mpc_main_count_time * inv_sec_day
            mpc_decyear = JdToDecyear(self.mpc_future_jd)

            # Dynamics update
            current_q_i2b, current_omega_b = self.mpc_update_attitude(last_omega_b,
                                                                      last_quaternion_q_i2b,
                                                                      last_torque_b)

            current_position_i, current_velocity_i = self.mpc_dynamics_orb.update_state_orbit(self.mpc_future_jd)
            current_sideral = gstime(self.mpc_future_jd)

            # ECI to Geodetic state
            alt, long, lat = self.eci_to_geodetic(current_position_i, current_sideral)

            # Get Earth magnetic field
            mag_b = self.get_mag_earth_b(mpc_decyear, alt, long, lat, current_q_i2b, current_sideral)
            #print(mag_b, ', Paso:', i + 1)

            # Control
            current_torque_b = np.zeros(3)

            # Save last data
            last_omega_b = current_omega_b
            last_quaternion_q_i2b = current_q_i2b
            last_torque_b = current_torque_b

        return

    def eci_to_geodetic(self, current_position_i, current_sideral):
        r = np.sqrt(current_position_i[0] ** 2 + current_position_i[1] ** 2)
        long = fmod2(np.arctan2(current_position_i[1], current_position_i[0]) - current_sideral)
        lat = np.arctan2(current_position_i[2], r)
        flag_iteration = True
        c = 1
        while flag_iteration:
            phi = lat
            c = 1 / np.sqrt(1 - earth_e2 * np.sin(phi) * np.sin(phi))
            lat = np.arctan2(current_position_i[2] + radius_earth * c
                             * earth_e2 * np.sin(phi) * 1000, r)
            if (np.abs(lat - phi)) <= geod_tolerance:
                flag_iteration = False

        alt = r / np.cos(lat) - radius_earth * c * 1000  # *metros

        if lat > np.pi / 2:
            lat -= twopi
        return alt, long, lat

    def mag_ned_to_eci(self, mag_0, theta, lonrad, gmst):
        mag_local_0y = rotationY(mag_0, np.pi - theta)
        mag_local_yz = rotationZ(mag_local_0y, -lonrad)
        return rotationZ(mag_local_yz, -gmst)

    def get_mag_earth_b(self, mpc_decyear, alt, long, lat, current_q_i2b, current_sideral):
        # Earth magnetic update in NED frame (North East Down)
        alt /= 1000
        """
         itype = 1 if geodetic(spheroid)
         itype = 2 if geocentric(sphere)
         alt   = height in km above sea level if itype = 1
               = distance from centre of Earth in km if itype = 2 (>3485 km)
        """
        x, y, z, f, gccolat = calculate_igrf(0, mpc_decyear, alt, lat, long, itype=1)
        mag_ned = [x, y, z]

        # NED to ECI
        mag_i = self.mag_ned_to_eci(mag_ned, gccolat, long, current_sideral)
        # Magnetic ECI to Body frame
        mag_b = current_q_i2b.frame_conv(mag_i)
        return mag_b

    def mpc_update_attitude(self, last_omega_b, last_quaternion_q_i2b, last_torque_b):
        q_i2b, omega_b = self.rungeonestep(last_omega_b, last_quaternion_q_i2b, last_torque_b)
        return q_i2b, omega_b

    def dynamics_and_kinematics(self, x, last_torque_b):
        x_omega_b = x[0:3]
        x_quaternion_i2b = x[3:]

        sk_omega = self.skewsymmetricmatrix(x_omega_b)
        o4k = self.omega4kinematics(x_omega_b)

        h_total_b = self.mpc_inertia.dot(x_omega_b)

        w_dot = -self.mpc_inv_inertia.dot(sk_omega.dot(h_total_b) - last_torque_b)

        q_dot = 0.5 * o4k.dot(x_quaternion_i2b)
        f_x = np.concatenate((w_dot, q_dot))
        return f_x

    def rungeonestep(self, last_omega_b, last_quaternion_q_i2b, last_torque_b):
        dt = self.mpc_sim_step_prop
        x = np.concatenate((last_omega_b, last_quaternion_q_i2b()))

        k1 = self.dynamics_and_kinematics(x, last_torque_b)
        xk2 = x + (dt / 2.0) * k1

        k2 = self.dynamics_and_kinematics(xk2, last_torque_b)
        xk3 = x + (dt / 2.0) * k2

        k3 = self.dynamics_and_kinematics(xk3, last_torque_b)
        xk4 = x + dt * k3

        k4 = self.dynamics_and_kinematics(xk4, last_torque_b)

        next_x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        omega_b = next_x[0:3]
        q_i2b = Quaternions(next_x[3:])
        q_i2b.normalize()
        return q_i2b, omega_b

    def calangmom(self, current_omega_b, current_quaternion_i2b):
        h_total_b = self.mpc_inertia.dot(current_omega_b)
        q_b2i = Quaternions(current_quaternion_i2b.conjugate())
        h_total_i = q_b2i.frame_conv(h_total_b)
        return h_total_b, h_total_i

    def skewsymmetricmatrix(self, x_omega_b):
        s_omega = np.zeros((3, 3))
        s_omega[1, 0] = x_omega_b[2]
        s_omega[2, 0] = -x_omega_b[1]
        s_omega[0, 1] = -x_omega_b[2]
        s_omega[0, 2] = x_omega_b[1]
        s_omega[2, 1] = x_omega_b[0]
        s_omega[1, 2] = -x_omega_b[0]
        return s_omega

    def omega4kinematics(self, x_omega_b):
        Omega = np.zeros((4, 4))
        Omega[1, 0] = -x_omega_b[2]
        Omega[2, 0] = x_omega_b[1]
        Omega[3, 0] = -x_omega_b[0]

        Omega[0, 1] = x_omega_b[2]
        Omega[0, 2] = -x_omega_b[1]
        Omega[0, 3] = x_omega_b[0]
        Omega[1, 2] = x_omega_b[0]
        Omega[1, 3] = x_omega_b[1]

        Omega[2, 1] = -x_omega_b[0]
        Omega[2, 3] = x_omega_b[2]

        Omega[3, 1] = -x_omega_b[1]
        Omega[3, 2] = -x_omega_b[2]
        return Omega
