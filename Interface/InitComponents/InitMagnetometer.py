
import configparser
import numpy as np


class InitMagnetometer(object):
    def __init__(self, path_com):
        config_com = configparser.ConfigParser()
        directory = path_com + 'magsensor.ini'

        config_com.read(directory, encoding="utf8")

        q_b2c = np.zeros(4)
        q_b2c[0] = config_com['MAGSENSOR1']['q_b2c(0)']
        q_b2c[1] = config_com['MAGSENSOR1']['q_b2c(1)']
        q_b2c[2] = config_com['MAGSENSOR1']['q_b2c(2)']
        q_b2c[3] = config_com['MAGSENSOR1']['q_b2c(3)']

        ScaleFactor = np.zeros((3, 3))
        ScaleFactor[0, 0] = config_com['MAGSENSOR1']['ScaleFactor(0)']
        ScaleFactor[0, 1] = config_com['MAGSENSOR1']['ScaleFactor(1)']
        ScaleFactor[0, 2] = config_com['MAGSENSOR1']['ScaleFactor(2)']
        ScaleFactor[1, 0] = config_com['MAGSENSOR1']['ScaleFactor(3)']
        ScaleFactor[1, 1] = config_com['MAGSENSOR1']['ScaleFactor(4)']
        ScaleFactor[1, 2] = config_com['MAGSENSOR1']['ScaleFactor(5)']
        ScaleFactor[2, 0] = config_com['MAGSENSOR1']['ScaleFactor(6)']
        ScaleFactor[2, 1] = config_com['MAGSENSOR1']['ScaleFactor(7)']
        ScaleFactor[2, 2] = config_com['MAGSENSOR1']['ScaleFactor(8)']

        Bias_c = np.zeros(3)
        Bias_c[0] = config_com['MAGSENSOR1']['Bias_c(0)']
        Bias_c[1] = config_com['MAGSENSOR1']['Bias_c(1)']
        Bias_c[2] = config_com['MAGSENSOR1']['Bias_c(2)']

        rw_stepwidth = float(config_com['MAGSENSOR1']['rw_stepwidth'])

        rw_stddev_c = np.zeros(3)
        rw_stddev_c[0] = config_com['MAGSENSOR1']['rw_stddev_c(0)']
        rw_stddev_c[1] = config_com['MAGSENSOR1']['rw_stddev_c(1)']
        rw_stddev_c[2] = config_com['MAGSENSOR1']['rw_stddev_c(2)']

        rw_limit_c = np.zeros(3)
        rw_limit_c[0] = config_com['MAGSENSOR1']['rw_limit_c(0)']
        rw_limit_c[1] = config_com['MAGSENSOR1']['rw_limit_c(1)']
        rw_limit_c[2] = config_com['MAGSENSOR1']['rw_limit_c(2)']

        nr_stddev_c = np.zeros(3)
        nr_stddev_c[0] = config_com['MAGSENSOR1']['nr_stddev_c(0)']
        nr_stddev_c[1] = config_com['MAGSENSOR1']['nr_stddev_c(1)']
        nr_stddev_c[2] = config_com['MAGSENSOR1']['nr_stddev_c(2)']

        current = float(config_com['MAGSENSOR1']['current'])

        self.mag_properties = {'q_b2c': q_b2c,
                                'ScaleFactor': ScaleFactor,
                                'Bias_c': Bias_c,
                                'rw_stepwidth': rw_stepwidth,
                                'rw_stddev_c': rw_stddev_c,
                                'rw_limit_c': rw_limit_c,
                                'nr_stddev_c': nr_stddev_c,
                                'current': current}
        print(' - Magnetometer added')
