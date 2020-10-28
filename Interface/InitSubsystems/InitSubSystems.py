
from ..InitComponents.InitComponents import InitComponents
from .InitADCS import InitADCS
from .InitCOMM import InitCOMM
import configparser


class InitSubSystems(object):
    def __init__(self, properties,  prop_step):
        self.system_name = ['CDH', 'ADCS', 'ODCS', 'POWER', 'COMM', 'STR', 'PAYLOAD', 'TCS']
        self.file_components = properties['path_com']

        self.components_name = {'ADCS': ['gyro', 'mag', 'rw', 'thruster', 'ss', 'stt', 'fss'],
                                'CDH': ['obc'],
                                'POWER': ['eps', 'sp', 'battery'],
                                'ODCS': [],
                                'COMM': ['ant'],
                                'STR': [],
                                'TCS': [],
                                'PAYLOAD': []}

        self.system_init_comp = {}
        self.system_init_setting = {}
        self.rewrite_init(properties)

        self.system_init_setting['ADCS']    = InitADCS(properties['adcs_setting']).get_setting()
        self.system_init_setting['COMM']    = InitCOMM(properties['comm_setting']).get_setting()

        #self.system_init_setting['ODCS']    = InitODCS(properties['odcs_setting'])
        #self.system_init_setting['CDH']     = InitCDH(properties['cdh_setting'])
        #self.system_init_setting['POWER']   = InitPOWER(properties['power_setting'])
        #self.system_init_setting['STR']     = InitSTR(properties['str_setting'])
        #self.system_init_setting['PAYLOAD'] = InitPayload(properties['payload_setting'])
        #self.system_init_setting['TCS']     = InitTCS(properties['tcs_setting'])

        self.init_components = {}
        for sub_elem in self.system_name:
            print("-----------------------------")
            print('SubSystem: ', sub_elem)
            self.init_components[sub_elem] = InitComponents(self.system_init_comp[sub_elem], prop_step)

    def rewrite_init(self, properties):
        for subsys in self.system_name:
            self.system_init_comp[subsys] = self.get_subsystems_components(properties[subsys.lower() + '_setting'],
                                                                           subsys)

    def get_subsystems_components(self, path, section):
        config = configparser.ConfigParser()
        config.read(path, encoding="utf8")

        properties_ = {'path_com': self.file_components}
        for name_comp in self.components_name[section]:
            properties_[name_comp+'_flag'] = config[section][name_comp] == 'True'
        return properties_

