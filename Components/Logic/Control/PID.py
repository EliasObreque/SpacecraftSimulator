
import numpy as np
REF_POINT = 2
NAD_POINT = 1
DETUMBLING = 0


class PID(object):
    def __init__(self, P, I, D, step):
        self.P = P
        self.I = I
        self.D = D
        self.error_int = 0
        self.step_width = step
        self.last_error = 0

    def set_gain(self, newP, newI, newD):
        self.P = newP
        self.I = newI
        self.D = newD

    def calc_control(self, error_, error_diff_, type_control):
        if type_control == DETUMBLING:
            error = error_diff_
            error_diff = (error - self.last_error)/self.step_width
            self.error_int = error_*self.step_width
            self.last_error = error
        elif type_control == NAD_POINT:
            error = error_
            error_diff = error_diff_
            self.error_int = error_ * self.step_width
        elif type_control == REF_POINT:
            error = error_
            error_diff = error_diff_
            self.error_int = error_ * self.step_width
        else:
            error = 0
            error_diff = 0
            self.error_int = 0
        ctrl = self.P.dot(error) + self.I.dot(self.error_int) + self.D.dot(error_diff)
        return ctrl

