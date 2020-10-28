
import configparser
import numpy as np


class InitFineSunSensor(object):
    def __init__(self, path_com):
        config_com = configparser.ConfigParser()
        directory = path_com + 'FSS.ini'

        config_com.read(directory, encoding="utf8")

        numb_sensor = int(config_com['SETTING']['numb_sensors'])
        self.fss_properties = []
        for i in range(1, numb_sensor + 1):
            name = 'FSUNSENSOR' + str(i)

            pos_vector = np.zeros(3)
            pos_vector[0] = config_com[name]['pos(0)']
            pos_vector[1] = config_com[name]['pos(1)']
            pos_vector[2] = config_com[name]['pos(2)']

            normal_vector = np.zeros(3)
            normal_vector[0] = config_com[name]['normal(0)']
            normal_vector[1] = config_com[name]['normal(1)']
            normal_vector[2] = config_com[name]['normal(2)']

            q_b2c = np.zeros(4)
            q_b2c[0] = config_com[name]['q_b2c(0)']
            q_b2c[1] = config_com[name]['q_b2c(1)']
            q_b2c[2] = config_com[name]['q_b2c(2)']
            q_b2c[3] = config_com[name]['q_b2c(3)']

            h = float(config_com[name]['h'])
            x0 = float(config_com[name]['x0'])
            y0 = float(config_com[name]['y0'])
            delta = float(config_com[name]['delta'])

            nr_stddev_c = float(config_com[name]['nr_stddev_c'])

            fss_prop = {'pos_vector': pos_vector,
                       'normal_vector': normal_vector,
                       'q_b2c': q_b2c,
                       'h': h,
                       'x0': x0,
                       'y0': y0,
                       'delta': delta,
                       'nr_stddev_c': nr_stddev_c}

            self.fss_properties.append(fss_prop)
            print(' - FineSunSensor' + str(i) + ' added')
