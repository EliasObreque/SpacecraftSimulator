
from .PID import PID
from .MPC import MPC


class Controller(object):
    def __init__(self):
        self.pid = PID
        self.mpc = MPC
