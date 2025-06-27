import constants as c
import numpy as np

class Airbrake:
    def __init__(self, max_rate):
        self.target_u = 0.0
        self.actual_u = 0.0
        self.max_rate = max_rate

    def set_target_u(self, target_u):
        self.target_u = target_u

    def update(self, dt):
        target_rate = (self.target_u - self.actual_u) / dt

        if abs(target_rate) > self.max_rate:
            self.actual_u += self.max_rate * dt
        else:
            self.actual_u = self.target_u

    def get_actual_u(self):
        return self.actual_u

    @staticmethod
    def get_a(u):
        return np.interp(u, c.u, c.a)

    @staticmethod
    def get_cd(u, mach):
        return np.interp(u, c.u, c.cd) * np.interp(mach, c.mach, c.cd_reduction)
