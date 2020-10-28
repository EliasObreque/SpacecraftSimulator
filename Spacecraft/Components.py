
from Components.AOCS.Gyro import Gyro
from Components.AOCS.Magnetometer import Magnetometer
from Components.AOCS.RWModel import RWModel
from Components.AOCS.SunSesnor import SunSensor
from Components.AOCS.FineSunSesnor import FineSunSensor
from Components.Abstract.ComponentBase import ComponentBase


class Components(ComponentBase):
    def __init__(self, data, dynamics, port_):
        ComponentBase.__init__(self, 5)
        self.data = data
        # general component
        # CDH
        self.obc        = None
        # AOCS
        self.gyro       = None
        self.mag        = None
        self.rwmodel    = None
        self.thruster   = None
        self.stt        = None
        self.sunsensors = None
        self.fss        = None
        self.gps        = None
        # POOWER
        self.eps        = None
        self.sp         = None
        self.battery    = None
        # COMM
        self.ant        = None

        self.get_list = []

        if data is not None:
            # For individual components
            # CDH

            if self.data.obc_properties is not None:
                g = 0
            else:
                del self.obc
                self.get_list.append(None)
            # ADCS

            if self.data.gyro_properties is not None:
                self.gyro = Gyro(port_, self.data.gyro_properties, dynamics)
                self.get_list.append(self.gyro)
            else:
                del self.gyro
                self.get_list.append(None)
            if self.data.mag_properties is not None:
                self.mag = Magnetometer(port_, self.data.mag_properties, dynamics)
                self.get_list.append(self.mag)
            else:
                del self.mag
                self.get_list.append(None)
            if self.data.rw_properties is not None:
                self.rwmodel = []
                for i in range(len(self.data.rw_properties)):
                    self.rwmodel.append(RWModel(self.data.rw_properties[i], dynamics))
                    self.get_list.append(self.rwmodel[i])
            else:
                del self.rwmodel
                self.get_list.append(None)
            if self.data.thruster_properties is not None:
                g = 0
            else:
                del self.thruster
                self.get_list.append(None)
            if self.data.stt_properties is not None:
                g = 0
            else:
                del self.stt
                self.get_list.append(None)
            if self.data.ss_properties is not None:
                self.sunsensors = []
                for i in range(len(self.data.ss_properties)):
                    self.sunsensors.append(SunSensor(port_, self.data.ss_properties[i], dynamics))
                    self.get_list.append(self.sunsensors[i])
                g = 0
            else:
                del self.sunsensors
            if self.data.fss_properties is not None:
                self.fss = []
                for i in range(len(self.data.fss_properties)):
                    self.fss.append(FineSunSensor(port_, self.data.fss_properties[i], dynamics))
                    self.get_list.append(self.fss[i])
                g = 0
            else:
                del self.fss
            # POWER

            if self.data.eps_properties is not None:
                g = 0
            else:
                del self.eps
                self.get_list.append(None)
            if self.data.sp_properties is not None:
                g = 0
            else:
                del self.sp
                self.get_list.append(None)
            if self.data.battery_properties is not None:
                g = 0
            else:
                del self.battery
                self.get_list.append(None)

