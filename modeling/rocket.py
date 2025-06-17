import math

import numpy as np
from modeling.environment import Environment
from modeling.airbrake import Airbrake
import constants as c

class Rocket:
    def __init__(self, environment: Environment, airbrake: Airbrake):
        self.state = np.array([0, 0, 0, 0]) # (x, z, v, theta)
        self.airbrake = airbrake
        self.environment = environment

    def set_u(self, u):
        self.airbrake.set_target_u(u)

    def get_u(self):
        return self.airbrake.get_actual_u()

    def state_derivative(self) -> np.ndarray:
        return np.array([
            self.state[2] * np.cos(self.state[3]),
            self.state[2] * np.sin(self.state[3]),
            -self.get_drag_force(self.state[2], self.state[1], self.get_u()) / c.post_burn_mass - c.gravity * np.sin(self.state[3]),
            -(c.gravity * np.cos(self.state[3])) / math.sqrt(self.state[2] ** 2 + c.velocity_buffer ** 2) #0.001 to prevent divide by zero error
        ])

    def project_next_state(self, dt) -> np.ndarray:
        state_derivative = self.state_derivative()
        return self.state + state_derivative * dt

    def set_state(self, state):
        self.state = state

    def update(self, dt):
        self.airbrake.update(dt)
        self.state = self.project_next_state(dt)

    def get_drag_force(self, v, z, u):
        return 0.5 * self.environment.get_p(z) * c.a(u) * c.cd(u) * v**2

    def is_apogee(self):
        return self.state[3] <= 0

