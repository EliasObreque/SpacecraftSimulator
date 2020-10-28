

class RWODE(object):
    def __init__(self, step_width, init_angular_velocity, target_angular_velocity, lag_coef):
        self.step_width = step_width
        self.init_angular_velocity = init_angular_velocity
        self.target_angular_velocity = target_angular_velocity
        self.lag_coef = lag_coef
        self.current_ang_velocity = self.init_angular_velocity
        self.current_time = 0

    def get_angular_velocity(self):
        return self.current_ang_velocity

    def dynamics_rw(self, state, t):
        rhs_ = (self.target_angular_velocity - state) / self.lag_coef
        return rhs_

    def update_ode(self):
        self.rungeonestep()

    def rungeonestep(self):
        x = self.current_ang_velocity
        t = self.current_time
        dt = self.step_width

        k1 = self.dynamics_rw(x, t)
        xk2 = x + (dt / 2.0) * k1

        k2 = self.dynamics_rw(xk2, (t + dt / 2.0))
        xk3 = x + (dt / 2.0) * k2

        k3 = self.dynamics_rw(xk3, (t + dt / 2.0))
        xk4 = x + dt * k3

        k4 = self.dynamics_rw(xk4, (t + dt))

        next_x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        self.current_time += self.step_width
        self.current_ang_velocity = next_x

    def set_target_angular_velocity(self, angular_velocity):
        self.target_angular_velocity = angular_velocity

    def set_lag_coef(self, lag_coef):
        self.lag_coef = lag_coef
