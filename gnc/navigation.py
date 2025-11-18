from dynamics import DynamicsModel
from constants import gnc_c as g

import numpy as np


class KalmanFilter:
    def __init__(self, dynamics_model: DynamicsModel):
        self.model = dynamics_model
        self.Q = g.Q
        self.R = g.R
        self.x = np.array([0, 0, 0, 0, 0]).T
        self.P = np.identity(5)

    def predict(self, dt, input: np.ndarray):
        A = self.model.get_kalman_A(dt)
        B = self.model.get_kalman_B(dt)

        self.x = A @ self.x + B @ input.T
        self.P = A @ self.P @ A.T + B @ self.Q @ B.T
    
    def update(self, output: np.ndarray):
        error = output - self.model.get_sensor_output(self.x.T)
        C = self.model.get_kalman_C(self.x.T)
        K = self.P @ C.T @ np.linalg.inv(C @ self.P @ C.T + self.R)

        self.x += K @ error.T
        self.P = (np.identity(5) - K @ C) @ self.P
    
    def get_optimal_state(self):
        return self.x.T
