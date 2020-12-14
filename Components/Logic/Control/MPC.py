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
from Library.math_sup.tools_reference_frame import rotationY, rotationZ, gstime, geodetic_to_ecef, eci_to_geodetic
from scipy.optimize import minimize
from scipy.optimize import Bounds


inv_sec_day = 1 / (60.0 * 60.0 * 24.0)
twopi = np.pi * 2
deg2rad = np.pi / 180.0
rad2deg = 1 / deg2rad
#  ------------ wgs-84 constants ------------
radius_earth = 6378.137  # km
earth_flat = 1.0 / 298.257223563
earth_e2 = earth_flat * (2 - earth_flat)
geod_tolerance = 1e-10  # rad
a2 = 40680631.6     # Equatorial radius
b2 = 40408296.0     # Polar radius


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

        # Target: Antenna Santaigo
        tar_alt = 572           # m
        tar_long = -70.6506     # degree
        tar_lat = -33.4372      # degree

        # Geodetic to ECEF
        self.tar_pos_ecef = geodetic_to_ecef(tar_alt, tar_long * deg2rad, tar_lat * deg2rad)
        # Vector direction of the Body frame to point to another vector
        self.b_dir = np.array([0, 0, 1])
        self.q_b2b_now2tar = Quaternions([0, 0, 0, 1])

    def open_loop1(self, current_att_state, current_jd):
        # Current state
        self.mpc_current_quaternion_i2b = current_att_state[0]
        self.mpc_current_omega_b = current_att_state[1]
        self.mpc_current_torque_b = current_att_state[2]

        self.mpc_start_jd = current_jd
        self.mpc_main_count_time = 0

        last_omega_b = self.mpc_current_omega_b
        last_quaternion_q_i2b = self.mpc_current_quaternion_i2b
        last_torque_b = self.mpc_current_torque_b

        last_pos_i, last_vel_i = self.mpc_dynamics_orb.update_state_orbit(current_jd)
        last_sideral = gstime(current_jd)
        last_tar_pos_eci = rotationZ(self.tar_pos_ecef, last_sideral)
        last_rel_vector = last_tar_pos_eci - last_pos_i

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
            current_tar_pos_eci_earth = rotationZ(self.tar_pos_ecef, current_sideral)

            # ECI to Geodetic state
            alt, long, lat = eci_to_geodetic(current_position_i, current_sideral)

            # Get Earth magnetic field
            mag_b = self.get_mag_earth_b(mpc_decyear, alt, long, lat, current_q_i2b, current_sideral)
            #print(mag_b, ', Paso:', i + 1)

            # Error determination

            current_tar_s2tar_i = current_tar_pos_eci_earth - current_position_i
            current_tar_pos_b = current_q_i2b.frame_conv(current_tar_s2tar_i)
            b_tar_b = current_tar_pos_b/np.linalg.norm(current_tar_pos_b)
            theta_e = np.arccos(np.dot(self.b_dir, b_tar_b))
            vec_u_e = np.cross(self.b_dir, b_tar_b)
            vec_u_e /= np.linalg.norm(vec_u_e)
            self.q_b2b_now2tar.setquaternion([vec_u_e, theta_e])
            self.q_b2b_now2tar.normalize()

            current_torque_b = np.zeros(3)

            # Save last data
            last_omega_b = current_omega_b
            last_quaternion_q_i2b = current_q_i2b
            last_torque_b = current_torque_b
            last_tar_pos_eci = current_tar_s2tar_i
        return

    def open_loop(self, current_att_state, current_jd):
        # Current state
        self.mpc_current_quaternion_i2b = current_att_state[0]
        self.mpc_current_omega_b = current_att_state[1]
        self.mpc_current_torque_b = current_att_state[2]

        self.mpc_start_jd = current_jd
        self.mpc_main_count_time = 0

        x0 = np.zeros(self.N_pred_horiz)+1
        xl = np.zeros(self.N_pred_horiz)
        xu = 1e-3*np.ones(self.N_pred_horiz)
        bounds = Bounds(xl, xu)
        res = minimize(self.objetive_function, x0, method='trust-constr',
                       options={'verbose': 1}, bounds=bounds)

        return res.x[0]

    def objetive_function(self, u):
        J = 0

        last_quaternion_q_i2b = self.mpc_current_quaternion_i2b
        last_omega_b = self.mpc_current_omega_b
        last_mag_torque_b = np.linalg.norm(self.mpc_current_torque_b)

        last_pos_i, last_vel_i = self.mpc_dynamics_orb.update_state_orbit(self.mpc_start_jd)
        last_sideral = gstime(self.mpc_start_jd)
        last_tar_pos_eci_earth = rotationZ(self.tar_pos_ecef, last_sideral)

        current_tar_s2tar_i = last_tar_pos_eci_earth - last_pos_i
        current_tar_pos_b = last_quaternion_q_i2b.frame_conv(current_tar_s2tar_i)
        b_tar_b = current_tar_pos_b / np.linalg.norm(current_tar_pos_b)
        theta_e = np.arccos(np.dot(self.b_dir, b_tar_b))
        vec_u_e = np.cross(self.b_dir, b_tar_b)
        vec_u_e /= np.linalg.norm(vec_u_e)

        for i in range(self.N_pred_horiz):
            print('Paso: ', i, 'Calculado')
            # current_torque_b * np.identity(3) * np.transpose(current_torque_b)
            # J += 0.1 * self.euclidea_dist(u[i*3:(i+1)*3], current_torque_b)**2
            J += 0.1 * theta_e**2 + 0.1 * u[i]**2 + 0.1 * (u[i] - last_mag_torque_b)**2
            current_torque_b = u[i] * vec_u_e

            self.mpc_main_count_time += self.mpc_sim_step_prop
            self.mpc_future_jd = self.mpc_start_jd + self.mpc_main_count_time * inv_sec_day

            mpc_decyear = JdToDecyear(self.mpc_future_jd)

            # Dynamics update
            current_q_i2b, current_omega_b = self.mpc_update_attitude(last_omega_b,
                                                                      last_quaternion_q_i2b,
                                                                      current_torque_b)

            current_position_i, current_velocity_i = self.mpc_dynamics_orb.update_state_orbit(self.mpc_future_jd)
            current_sideral = gstime(self.mpc_future_jd)
            current_tar_pos_eci_earth = rotationZ(self.tar_pos_ecef, current_sideral)

            # ECI to Geodetic state
            alt, long, lat = eci_to_geodetic(current_position_i, current_sideral)

            # Get Earth magnetic field
            mag_b = self.get_mag_earth_b(mpc_decyear, alt, long, lat, current_q_i2b, current_sideral)
            #print(mag_b, ', Paso:', i + 1)

            # Error determination
            current_tar_s2tar_i = current_tar_pos_eci_earth - current_position_i
            current_tar_pos_b = current_q_i2b.frame_conv(current_tar_s2tar_i)
            b_tar_b = current_tar_pos_b / np.linalg.norm(current_tar_pos_b)
            theta_e = np.arccos(np.dot(self.b_dir, b_tar_b))
            vec_u_e = np.cross(self.b_dir, b_tar_b)
            vec_u_e /= np.linalg.norm(vec_u_e)

            # Save last data
            last_omega_b = current_omega_b
            last_quaternion_q_i2b = current_q_i2b
            last_mag_torque_b = u[i]
        return J

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

    def euclidea_dist(self, x, y):
        return np.sqrt((x[0]-y[0])**2+(x[1]-y[1])**2+(x[1]-y[1])**2)

