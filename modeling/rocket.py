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

    def state_acceleration(self, u) -> np.ndarray:
        v = np.array([self.state[2], self.state[3]])
        v_mag = np.linalg.norm(v)
        v_unit = v / (v_mag + 0.0000001) #added to prevent divide by zero errors
        f_drag = self.get_drag_force(v_mag, self.state[1], u)

        return -v_unit * f_drag + np.array([0, -c.gravity])

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
        return 0.5 * self.environment.get_p(z) * Airbrake.get_a(u) * Airbrake.get_cd(u) * v**2

    def is_apogee(self):
        return self.state[3] <= 0
