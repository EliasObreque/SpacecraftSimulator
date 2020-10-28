
import configparser
import numpy as np


class InitSunSensor(object):
    def __init__(self, path_com):
        config_com = configparser.ConfigParser()
        directory = path_com + 'SS.ini'

        config_com.read(directory, encoding="utf8")

        numb_sensor = int(config_com['SETTING']['numb_sensors'])
        self.ss_properties = []
        for i in range(1, numb_sensor + 1):
            name = 'SUNSENSOR' + str(i)

            pos_vector = np.zeros(3)
            pos_vector[0] = config_com[name]['pos(0)']
            pos_vector[1] = config_com[name]['pos(1)']
            pos_vector[2] = config_com[name]['pos(2)']

            normal_vector = np.zeros(3)
            normal_vector[0] = config_com[name]['normal(0)']
            normal_vector[1] = config_com[name]['normal(1)']
            normal_vector[2] = config_com[name]['normal(2)']

            cosine_error = float(config_com[name]['cosine_error'])

            I_max = float(config_com[name]['I_max'])

            nr_stddev_c = float(config_com[name]['nr_stddev_c'])

            ss_prop = {'pos_vector': pos_vector,
                       'normal_vector': normal_vector,
                       'cosine_error': cosine_error,
                       'I_max': I_max,
                       'nr_stddev_c': nr_stddev_c}

            self.ss_properties.append(ss_prop)
            print(' - SunSensor' + str(i) + ' added')
