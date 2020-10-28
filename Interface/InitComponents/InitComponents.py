
from .InitGyro import InitGyro
from .InitMagnetometer import InitMagnetometer
from .InitRWModel import InitRWModel
from .InitSunSesnor import InitSunSensor
from .InitFineSunSensor import InitFineSunSensor


class InitComponents(object):
    def __init__(self, properties,  prop_step):

        self.gyro_properties        = None
        self.mag_properties         = None
        self.obc_properties         = None
        self.rw_properties          = None
        self.eps_properties         = None
        self.sp_properties          = None
        self.battery_properties     = None
        self.thruster_properties    = None
        self.stt_properties         = None
        self.ss_properties          = None
        self.fss_properties         = None
        self.gps_properties         = None
        self.temp_properties        = None
        self.thermal_properties     = None
        self.camera_properties      = None
        self.ant_properties         = None

        path = properties['path_com']

        comp_in_subsystem = list(properties.keys())[1:]

        for comp_flag in comp_in_subsystem:
            if comp_flag[:-5] == 'gyro' and properties['gyro_flag']:
                self.gyro_properties = InitGyro(path).gyro_properties
            elif comp_flag[:-5] == 'mag' and properties['mag_flag']:
                self.mag_properties = InitMagnetometer(path).mag_properties
            elif comp_flag[:-5] == 'obc' and properties['obc_flag']:
                k = 0
            elif comp_flag[:-5] == 'power' and properties['power_flag']:
                k = 0
            elif comp_flag[:-5] == 'rw' and properties['rw_flag']:
                self.rw_properties = InitRWModel(path,  prop_step).rw_properties
            elif comp_flag[:-5] == 'thruster' and properties['thruster_flag']:
                k = 0
            elif comp_flag[:-5] == 'stt' and properties['stt_flag']:
                k = 0
            elif comp_flag[:-5] == 'ss' and properties['ss_flag']:
                self.ss_properties = InitSunSensor(path).ss_properties
            elif comp_flag[:-5] == 'fss' and properties['fss_flag']:
                self.fss_properties = InitFineSunSensor(path).fss_properties
