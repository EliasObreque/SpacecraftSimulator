
import numpy as np
from .AirDrag import AirDrag
from .GravGrad import GravGrad
from .MagDist import Magnetic
from .SRP import SRP


class Disturbances(object):
    def __init__(self, disturbance_properties, environment, spacecraft):
        self.dist_environment = environment
        self.dist_spacecraft = spacecraft

        print('\nDisturbances properties')
        print('------------------------------')
        self.disturbance_ = []
        if disturbance_properties['GRA']['gra_calculation']:
            grav = GravGrad(disturbance_properties['GRA'], self.dist_spacecraft)
            print('Gravitational: ' + str(grav.dist_flag))
            self.disturbance_.append(grav)
        if disturbance_properties['ATM']['atm_calculation']:
            atmd = AirDrag(disturbance_properties['ATM'], disturbance_properties['SFF'])
            print('Atmosphere: ' + str(atmd.dist_flag))
            self.disturbance_.append(atmd)
        if disturbance_properties['MAG']['mag_calculation']:
            mag = Magnetic(disturbance_properties['MAG'])
            print('Magnetic: ' + str(mag.dist_flag))
            self.disturbance_.append(mag)
        print('------------------------------')

        self.dist_torque_b = np.zeros(3)
        self.dist_force_b  = np.zeros(3)

    def update(self):
        self.reset_output()
        for dist in self.disturbance_:
            if dist.dist_flag:
                dist.update(self.dist_environment, self.dist_spacecraft)
                self.dist_torque_b += dist.get_torque_b()
                self.dist_force_b += dist.get_force_b()

    def get_dist_torque(self):
        return self.dist_torque_b

    def get_dis_force(self):
        return self.dist_force_b

    def reset_output(self):
        self.dist_torque_b = np.zeros(3)
        self.dist_force_b  = np.zeros(3)
