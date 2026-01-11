from gnc.dynamics import DynamicsModel
from constants import gnc_c as g
from constants import rocket_c as r

import numpy as np


class KalmanFilter:
    def __init__(self, dynamics_model: DynamicsModel):
        self.model = dynamics_model
        self.actuator_position = 0
        self.nav_x = np.array([0, 0, 0, 0]).T
        self.drag_x = np.array([r.rocket_cd_airbrake[0], r.rocket_cd_airbrake[1], r.rocket_cd_airbrake[2]]).T
        self.nav_P = np.zeros((4, 4))
        self.drag_P = np.zeros((3, 3))

    def predict(self, dt, nav_input: np.ndarray):
        A = self.model.get_kalman_A(dt)
        B = self.model.get_kalman_B(dt)
        
        Q = g.q * np.array([
            [0.333 * dt ** 3, 0.5 * dt ** 2, 0, 0],
            [0.5 * dt ** 2, dt, 0, 0],
            [0, 0, 0.333 * dt ** 3, 0.5 * dt ** 2],
            [0, 0, 0.5 * dt ** 2, dt]
        ])

        self.nav_x = A @ self.nav_x + B @ nav_input.T
        self.nav_P = A @ self.nav_P @ A.T + Q
    
    def predict_drag(self, dt):
        A = self.model.get_drag_A()
        Q = g.dQ * dt

        self.drag_x = A @ self.drag_x
        self.drag_P = A @ self.drag_P @ A.T + Q
    
    def update(self, nav_output: np.ndarray):
        error = nav_output - self.nav_x.T[2]
        C = np.array([[0, 0, 1, 0]])

        K = self.nav_P @ C.T @ np.linalg.inv(C @ self.nav_P @ C.T + g.R)

        self.nav_x += K @ error.T
        self.nav_P = (np.identity(4) - K @ C) @ self.nav_P
    
    def update_drag(self, drag_output: np.ndarray):
        drag_input = np.append(self.nav_x.T, self.actuator_position)
        error = drag_output - self.model.get_drag_output(self.drag_x.T, drag_input)

        J_state, J_input = self.model.get_linearized_drag_output(self.drag_x.T, drag_input)
        C = J_state.T
        D = J_input.T

        K = self.drag_P @ C.T @ np.linalg.inv(C @ self.drag_P @ C.T + g.dR)
        self.drag_x += K @ error.T
        self.drag_P = (np.identity(3) - K @ C) @ self.drag_P
    
    def update_actuator(self, U_actual):
        self.actuator_position = U_actual
    
    def get_optimal_nav_state(self):
        return self.nav_x.T
    
    def get_optimal_drag_state(self):
        return self.drag_x.T
    
    def get_optimal_state(self):
        return self.model.get_state(self.nav_x.T, self.drag_x.T, self.actuator_position)
