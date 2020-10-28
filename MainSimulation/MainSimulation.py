"""
Created on Wed Jan 15 11:19:53 2020

@author: EO
"""

from Spacecraft.Spacecraft import Spacecraft
from GroundStation.GroundStation import GroundStation
from Dynamics.SimTime import SimTime
from Interface.Initializer import InitialConfig
from Interface.Logger import Logger
from Environments.Environment import Environment
from Disturbance.Disturbances import Disturbances

import numpy as np
import pandas as pd
import datetime
import os

twopi   = 2.0 * np.pi
deg2rad = np.pi / 180.0
rad2deg = 1 / deg2rad


class MainSimulation(InitialConfig, Logger):
    def __init__(self):

        InitialConfig.__init__(self)
        Logger.__init__(self, self.logger_properties)

        self.simtime = SimTime(self.time_properties)
        self.spacecraft = Spacecraft(self.spacecraft_properties, self.components_properties, self.simtime)
        self.groundstation = GroundStation(self.groundstation_properties, self.spacecraft.dynamics)

        self.environment = Environment(self.environment_properties)
        self.disturbance = Disturbances(self.disturbance_properties, self.environment, self.spacecraft)

        # Auxiliary variables
        date = datetime.datetime.now()
        self.filename = date.strftime('%Y-%m-%d %H-%M-%S')

    def run_simulation(self):
        self.spacecraft.dynamics.orbit.set_propagator()
        # Loop
        self.simtime.reset_countTime()
        print('Simulation running...')
        while self.simtime.maincountTime <= self.simtime.endsimTime:
            # spacecraft update
            self.spacecraft.update()

            # current Environment and disturbances
            self.environment.update(self.simtime.current_decyaer, self.spacecraft.dynamics)
            self.disturbance.update()

            # Add the force and torque generated by the disturbance for the next dynamics propagation
            self.spacecraft.dynamics.attitude.add_ext_torque_b(self.disturbance.get_dist_torque())
            self.spacecraft.dynamics.add_ext_force_b(self.disturbance.get_dis_force())

            # Add the force and torque generated by the satellite for the next dynamics propagation
            self.spacecraft.dynamics.attitude.add_int_torque_b(self.spacecraft.generate_torque_b())

            # Set magnetic vector calculated by the environment for ADCS estimation
            if self.spacecraft.subsystems['ADCS'] is not None:
                self.spacecraft.subsystems['ADCS'].set_magVector(self.environment.magnetic.Mag_i,
                                                                 self.environment.magnetic.Mag_b)

            if self.simtime.log_flag:
                self.spacecraft.update_data()
                self.simtime.progressionsimTime()
                self.simtime.log_flag = False

            # update time
            self.simtime.updateSimtime()

        # Data report to create dictionary
        self.spacecraft.create_report()
        #self.ephemeris.earth.create_report()

        # Save Dataframe pandas in csv file
        self.save_data()
        print('Simulation Finished')

    def save_data(self):
        master_data = self.spacecraft.master_data_satellite
        database = pd.DataFrame(master_data, columns=master_data.keys())
        if os.path.isdir("./Data/logs/") is False:
            os.mkdir("./Data/logs/")
        database.to_csv("./Data/logs/"+self.filename+".csv", index=False, header=True)
        print("Data saved to file:")
        print(self.filename)
