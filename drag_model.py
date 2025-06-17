import math

import numpy as np
import constants as const


def get_reduction_factor(height):
    return 1.0 / math.exp((const.gravity * height) / (const.specific_gas_constant * const.temp_0))

def get_density(height):
    return get_reduction_factor(height) * const.density_0

def get_drag(drag_constant, velocity, height):
        return drag_constant * get_reduction_factor(height) * math.pow(velocity, 2)


class DragModel:
    def __init__(self):
        self.k1 = 0.5 * const.density_0 * const.cda / const.post_burn_mass
        self.k2 = 0.5 * const.density_0 * const.cda_rate / const.post_burn_mass
        self.drag_data = np.array([])

    def get_drag_constant(self, airbrake_input):
        return self.k1 + self.k2 * airbrake_input

    def get_airbrake_input(self, drag_constant):
        return (drag_constant - self.k1) / self.k2






