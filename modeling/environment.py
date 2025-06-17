import math

import constants as c

class Environment:
    def __init__(self, p0, t0, molar_mass, lapse_rate):
        self.p0 = p0
        self.t0 = t0
        self.lapse_rate = lapse_rate
        self.molar_mass = molar_mass
        self.sgc = c.gas_constant / molar_mass

    def set_p0(self, new_p0, weight):
        self.p0 = (1 - weight) * self.p0 + weight * new_p0

    def set_t0(self, new_t0, weight):
        self.t0 = (1 - weight) * self.t0 + weight * new_t0

    def get_d0(self):
        return self.p0 / (self.t0 * self.sgc)

    def get_p(self, z):
        return self.p0 * math.pow(
            1 - (self.lapse_rate * z) / self.t0,
            (c.gravity * self.molar_mass) / (self.sgc * self.lapse_rate)
        )

    def get_t(self, z):
        return self.t0 - self.lapse_rate * z

    def get_d(self, z):
        return self.get_p(z) / (self.get_t(z) * self.sgc)

    def get_z(self, p):
        return self.t0 / self.lapse_rate * (1 - math.pow(p / self.p0, (self.sgc * self.lapse_rate) / (c.gravity * self.molar_mass)))




