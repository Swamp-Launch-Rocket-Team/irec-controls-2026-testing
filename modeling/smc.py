from unittest import case

import numpy as np

import constants as c


class SMC:
    def __init__(self, target_apogee, target_input, surface, convergence_time, mode="saturation"):
        self.target_apogee = target_apogee
        self.target_input = target_input
        self.surface = surface
        self.mode = mode.lower()
        self.convergence_time = convergence_time

    def get_output(self, state, authority_rate):
        thickness = self.get_boundary_thickness(state, authority_rate)
        sliding_z_dot: float = self.surface(np.array([[state[1], state[2]]]))[0]
        error = state[3] - sliding_z_dot

        if abs(error) < thickness:
            if self.mode == "saturation":
                return self.get_saturation_output(error, thickness)
            elif self.mode == "deadband":
                return self.get_deadband_output()
            else:
                raise ValueError(f"Invalid mode: {self.mode}")
        else:
            if error > 0.0:
                return 1.0
            else:
                return 0.0


    @staticmethod
    def get_time_to_apogee(state):
        return state[3] / c.gravity

    @staticmethod
    def get_boundary_thickness_0(authority_rate):
        lag_time = 0.5 / c.max_rate
        return lag_time * authority_rate

    def get_convergence_multiplier(self, state):
        t_apogee = self.get_time_to_apogee(state)

        if t_apogee > c.convergence_time:
            return 1.0
        elif t_apogee > 0.0:
            return t_apogee / c.convergence_time
        else:
            return 0.0

    def get_boundary_thickness(self, state, authority_rate):
        multiplier = self.get_convergence_multiplier(state)
        thickness_0 = self.get_boundary_thickness_0(state)
        return multiplier * thickness_0

    def get_saturation_output(self, error, thickness):
        return self.target_input + error / (2.0 * thickness)

    def get_deadband_output(self):
        return self.target_input
