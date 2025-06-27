import math

import numpy as np
from environment import Environment

from airbrake import Airbrake
import constants as c


class Rocket:
    def __init__(self, environment: Environment, airbrake: Airbrake):
        self.state = np.array([0, 0, 0, 0]) # (x, z, x-dot, z-dot)
        self.airbrake = airbrake
        self.environment = environment

    def set_u(self, u):
        self.airbrake.set_target_u(u)

    def get_u(self):
        return self.airbrake.get_actual_u()

    def get_v(self):
        return np.array([self.state[2], self.state[3]])

    def get_v_mag(self):
        return np.linalg.norm(self.get_v())

    def get_unit_v(self):
        return self.get_v() / (self.get_v_mag() + 0.0000001)

    def state_acceleration(self, u) -> np.ndarray:
        f_drag = self.get_drag_force(self.get_v_mag(), self.state[1], u)
        return -self.get_unit_v() * f_drag + np.array([0, -c.gravity])

    def get_authority_rate(self):
        return abs(self.state_acceleration(1.0)[1] - self.state_acceleration(0.0)[1])

    def project_next_state(self, u, dt) -> np.ndarray:
        state_acceleration = self.state_acceleration(u) #acceration as an input.
        A = self.get_A(dt)
        B = self.get_B(dt)

        state_column = A @ self.state + B @ state_acceleration
        return state_column.reshape(-1)


    @staticmethod
    def get_A(dt):
        return np.array([[1.0, 0.0, dt, 0.0],
                        [0.0, 1.0, 0.0, dt],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0]])

    @staticmethod
    def get_B(dt):
        return np.array([[0.5*dt**2, 0.0],
                        [0.0, 0.5*dt**2],
                        [dt, 0.0],
                        [0.0, dt]])

    def set_state(self, state):
        self.state = state

    def update(self, dt):
        self.airbrake.update(dt)
        self.state = self.project_next_state(dt, self.get_u())

    def get_drag_force(self, v, z, u):
        return 0.5 * self.environment.get_p(z) * Airbrake.get_a(u) * Airbrake.get_cd(u, v / self.environment.get_c(z)) * v**2

    def is_apogee(self):
        return self.state[3] <= 0
