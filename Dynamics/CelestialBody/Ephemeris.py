# -*- coding: utf-8 -*-
"""
Created by:

@author: Elias Obreque
els.obrq@gmail.com

----------------------------------------------------
2287184.50..2688976.50  Solar System Barycenter (0) -> Mercury Barycenter (1)
2287184.50..2688976.50  Solar System Barycenter (0) -> Venus Barycenter (2)
2287184.50..2688976.50  Solar System Barycenter (0) -> Earth Barycenter (3)
2287184.50..2688976.50  Solar System Barycenter (0) -> Mars Barycenter (4)
2287184.50..2688976.50  Solar System Barycenter (0) -> Jupiter Barycenter (5)
2287184.50..2688976.50  Solar System Barycenter (0) -> Saturn Barycenter (6)
2287184.50..2688976.50  Solar System Barycenter (0) -> Uranus Barycenter (7)
2287184.50..2688976.50  Solar System Barycenter (0) -> Neptune Barycenter (8)
2287184.50..2688976.50  Solar System Barycenter (0) -> Pluto Barycenter (9)
2287184.50..2688976.50  Solar System Barycenter (0) -> Sun (10)
2287184.50..2688976.50  Earth Barycenter (3) -> Moon (301)
2287184.50..2688976.50  Earth Barycenter (3) -> Earth (399)
2287184.50..2688976.50  Mercury Barycenter (1) -> Mercury (199)
2287184.50..2688976.50  Venus Barycenter (2) -> Venus (299)
----------------------------------------------------

ref:
 - https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/
 - https://pypi.org/project/jplephem/
"""
from .Earth import Earth
from .Mars import Mars
from .Moon import Moon
from .Sun import Sun
import numpy as np
from jplephem.spk import SPK
from pathlib import Path

try:
    KERNEL_SPK = SPK.open(str(Path("./Dynamics/CelestialBody/cspice/generic_kernels/spk/planets/de430.bsp")))
except:
    print('File de430.bsp not found, download and copy to directory.\n')
    print('Link: https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/')


class Ephemeris(object):
    def __init__(self, ephemerides_properties):
        self.inertial_frame = ephemerides_properties['inertial_frame']
        self.aberration_correction = ephemerides_properties['aberration_correction']
        self.center_object = ephemerides_properties['center_object']
        self.num_of_selected_body = ephemerides_properties['num_of_selected_body']

        self.list_body = ephemerides_properties['selected_body']

        self.center_object_pos_i = np.zeros(3)
        self.center_object_vel_i = np.zeros(3)

        self.selected_body = {'EARTH': Earth(KERNEL_SPK),
                              'SUN': Sun(KERNEL_SPK),
                              'MOON': Moon(KERNEL_SPK),
                              'MARS': Mars(KERNEL_SPK)}

        self.center_object_mu = self.selected_body[self.center_object].mu

        max_body = len(self.selected_body)
        list_body = [*self.selected_body]
        for i in range(self.num_of_selected_body, max_body):
            del self.selected_body[list_body[i]]

    def update(self, current_jd, sc_pos_from_center_obj_i, sc_vel_from_center_obj_i, q_i2b):
        self.center_object_pos_i, self.center_object_vel_i = self.selected_body[self.center_object].update_state(
            current_jd, np.zeros(3), np.zeros(3), sc_pos_from_center_obj_i, sc_vel_from_center_obj_i, q_i2b,
            iscenter=True)
        for body in self.list_body:
            self.selected_body[body].update_state(current_jd, self.center_object_pos_i, self.center_object_vel_i,
                                                  sc_pos_from_center_obj_i, sc_vel_from_center_obj_i, q_i2b)

    def save_ephemeris_data(self):
        for body in self.list_body:
            self.selected_body[body].save_data()

    def get_log_values(self):
        report = {}
        for body in self.list_body:
            report = {**report, **self.selected_body[body].get_log_values()}
        return report
