
from Interface.InitSubsystems.InitSubSystems import InitSubSystems
from Components.Abstract.ADCS import ADCS
from Components.Abstract.ODCS import ODCS
from Components.Abstract.COMM import COMM

import numpy as np


class SubSystems(InitSubSystems):
    def __init__(self, components_properties, dynamics, prop_step):
        InitSubSystems.__init__(self, components_properties,  prop_step)

        cdh = None
        if components_properties['create_adcs']:
            adcs = ADCS(self.init_components['ADCS'], self.system_init_setting['ADCS'], dynamics)
        else:
            adcs = None
        if components_properties['create_odcs']:
            odcs = ODCS(self.init_components['ODCS'], dynamics)
        else:
            odcs = None
        if components_properties['create_power']:
            power = None    # por hacer
        else:
            power = None
        if components_properties['create_comm']:
            comm = COMM(self.init_components['COMM'], dynamics)
        else:
            comm = None
        str = None
        payload = None
        tcs = None

        self.subsystems = {'CDH': cdh,
                           'ADCS': adcs,
                           'ODCS': odcs,
                           'POWER': power,
                           'COMM': comm,
                           'STR': str,
                           'PAYLOAD': payload,
                           'TCS': tcs}

    def generate_torque_b(self):
        if self.subsystems['ADCS'] is not None:
            return self.subsystems['ADCS'].get_rwtorque()
        else:
            return np.zeros(3)

    def generate_force_b(self):
        if self.subsystems['ODCS'] is not None:
            return self.subsystems['ODCS'].get_force()
        else:
            return np.zeros(3)

    def save_log_values(self):
        for sub_elem in self.system_name:
            if self.subsystems[sub_elem] is not None:
                for comp in self.subsystems[sub_elem].components.get_list:
                    if comp is not None:
                        comp.log_value()
