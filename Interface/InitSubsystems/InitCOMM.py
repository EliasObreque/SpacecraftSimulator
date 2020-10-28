
import configparser
import numpy as np


class InitCOMM(object):
    def __init__(self, file_path):

        section = 'SETTING'
        config = configparser.ConfigParser()
        config.read(file_path, encoding="utf8")

        self.properties_ = {'port_id': float(config[section]['port_id']),
                            'COMM_COMPONENT_NUMBER': float(config[section]['COMM_COMPONENT_NUMBER']),
                            'COMM_com_frequency': float(config[section]['COMM_com_frequency'])}

    def get_setting(self):
        return self.properties_

