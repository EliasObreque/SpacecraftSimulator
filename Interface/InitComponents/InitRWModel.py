
import configparser
import numpy as np


class InitRWModel(object):
    def __init__(self, path_com, prop_step):
        config_com = configparser.ConfigParser()
        directory = path_com + 'RW.ini'
        config_com.read(directory, encoding="utf8")

        setting = str(config_com['SETTING']['config_rw'])

        self.rw_properties = []
        for id_conifg in setting:
            rw_number = 'RW' + id_conifg

            torque_transition = np.zeros(3)
            torque_transition[0] = config_com[rw_number]['torque_transition(0)']
            torque_transition[1] = config_com[rw_number]['torque_transition(1)']
            torque_transition[2] = config_com[rw_number]['torque_transition(2)']
            rw_properties_id   = {'inertia': config_com[rw_number]['inertia'],
                                  'angular_velocity_init': config_com[rw_number]['angular_velocity_init'],
                                  'angular_velocity_upperlimit_init': config_com[rw_number]['angular_velocity_upperlimit_init'],
                                  'angular_velocity_lowerlimit_init': config_com[rw_number]['angular_velocity_lowerlimit_init'],
                                  'motor_drive_init': config_com[rw_number]['motor_drive_init'],
                                  'torque_transition': torque_transition,
                                  'firstorder_lag_const': config_com[rw_number]['firstorder_lag_const'],
                                  'dead_time': config_com[rw_number]['dead_time'],
                                  'rw_number_id': rw_number,
                                  'max_torque': config_com[rw_number]['max_torque'],
                                  'prop_step': prop_step}
            self.rw_properties.append(rw_properties_id)
            print(' - RW ' + id_conifg + ' added')




