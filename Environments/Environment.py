
from .MagEnv import MagEnv
from .Atmosphere import Atmosphere
from .SolarRadiation import SolarRadiation


class Environment(object):
    def __init__(self, environment_properties):
        self.envir = []
        print('\nEnvironment properties')
        print('------------------------------')
        if environment_properties['ATM']['atm_calculation']:
            self.atm = Atmosphere(environment_properties['ATM'])
            self.envir.append(self.atm)
            print('Atmosphere: ' + str(self.atm.envir_flag))
        if environment_properties['MAG']['mag_calculation']:
            self.magnetic = MagEnv(environment_properties['MAG'])
            self.envir.append(self.magnetic)
            print('Magnetic: ' + str(self.magnetic.envir_flag))
        if environment_properties['SRP']['srp_calculation']:
            self.srp = SolarRadiation(environment_properties['SRP'])
            self.envir.append(self.srp)
            print('Solar radiation: ' + str(self.srp.envir_flag))
        print('------------------------------')

    def update(self, decyear, dynamics):
        for env in self.envir:
            if env.envir_flag:
                env.update(dynamics, decyear)
