
import configparser
import numpy as np


class InitMTT(object):
    def __init__(self, path_com, prop_step):
        config_com = configparser.ConfigParser()
        directory = path_com + 'MTT.ini'
        config_com.read(directory, encoding="utf8")

        setting = str(config_com['SETTING']['config_mtt'])

        for id_conifg in setting:
            mtt_number = 'MAGTORQUER' + id_conifg

            q_b2c = np.zeros(4)
            q_b2c[0] = config_com[mtt_number]['q_b2c(0)']
            q_b2c[1] = config_com[mtt_number]['q_b2c(1)']
            q_b2c[2] = config_com[mtt_number]['q_b2c(2)']
            q_b2c[3] = config_com[mtt_number]['q_b2c(3)']

            max_c_am2   = np.zeros(3)
            min_c_am2   = np.zeros(3)
            bias_c      = np.zeros(3)
            rw_stddev_c = np.zeros(3)
            rw_limit_c  = np.zeros(3)
            nr_stddev_c = np.zeros(3)
            for i in range(3):
                max_c_am2[i] = config_com[mtt_number]['Max_c('+str(i)+')']
                min_c_am2[i] = config_com[mtt_number]['Min_c('+str(i)+')']
                bias_c[i] = config_com[mtt_number]['Bias_c('+str(i)+')']
                rw_stddev_c[i] = config_com[mtt_number]['rw_stddev_c('+str(i)+')']
                rw_limit_c[i] = config_com[mtt_number]['rw_limit_c(' + str(i) + ')']
                nr_stddev_c[i] = config_com[mtt_number]['nr_stddev_c(' + str(i) + ')']

            rw_stepwidth = float(config_com[mtt_number]['rw_stepwidth'])

            rw_properties_id   = {'q_b2c': q_b2c,
                                  'prop_step': rw_stepwidth,
                                  'max_c_am2': max_c_am2,
                                  'min_c_am2': min_c_am2,
                                  'bias_c': bias_c,
                                  'rw_stddev_c': rw_stddev_c,
                                  'rw_limit_c': rw_limit_c,
                                  'nr_stddev_c': nr_stddev_c}
            self.mtt_properties = rw_properties_id
            print(' - MTT ' + id_conifg + ' added')




