from gnc.dynamics import DynamicsModel
from constants import gnc_c as g
from constants import rocket_c as r

import numpy as np


class KalmanFilter:
    def __init__(self, dynamics_model: DynamicsModel):
        self.model = dynamics_model
        self.q = g.q
        self.R = g.R
        self.R_drag = g.R_drag
        self.nav_x = np.array([0, 0, 0, 0, r.rocket_cd_airbrake[0], r.rocket_cd_airbrake[1]]).T
        self.U = 0
        self.P = np.zeros((6, 6))

    def predict(self, dt, input: np.ndarray):
        A = self.model.get_kalman_A(dt)
        B = self.model.get_kalman_B(dt)
        
        Q = self.q * np.array([
            [0.333 * dt ** 3, 0.5 * dt ** 2, 0, 0, 0, 0],
            [0.5 * dt ** 2, dt, 0, 0, 0, 0],
            [0, 0, 0.333 * dt ** 3, 0.5 * dt ** 2, 0, 0],
            [0, 0, 0.5 * dt ** 2, dt, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]
        ])

        Q += g.Q_cd * dt

        self.nav_x = A @ self.nav_x + B @ input.T
        self.P = A @ self.P @ A.T + Q
    
    def update(self, output: np.ndarray, input: np.ndarray):
        error = output - self.model.get_sensor_output(self.nav_x.T, input)
        error[1] = 0.0
        error[2] = 0.0

        J_state, J_input = self.model.get_linearized_output(self.nav_x.T, input)
        C = J_state.T
        D = J_input.T

        K = self.P @ C.T @ np.linalg.inv(C @ self.P @ C.T + self.R)

        self.nav_x += K @ error.T
        self.P = (np.identity(6) - K @ C) @ self.P
    
    def update_with_drag(self, output: np.ndarray, input: np.ndarray):
        error = output - self.model.get_sensor_output(self.nav_x.T, input)
        J_state, J_input = self.model.get_linearized_output(self.nav_x.T, input)
        C = J_state.T
        D = J_input.T

        K = self.P @ C.T @ np.linalg.inv(C @ self.P @ C.T + self.R_drag)

        self.nav_x += K @ error.T
        self.P = (np.identity(6) - K @ C) @ self.P
    
    def get_optimal_nav_state(self):
        return self.nav_x.T
    
    def update_actuator(self, U_actual):
        self.U = U_actual
    
    def get_optimal_state(self):
        return self.model.get_state(self.nav_x.T, self.U)
