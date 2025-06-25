import numpy as np
from ahrs import AHRS
from altimeter import Altimeter
from modeling.rocket import Rocket


class Kalman:
    def __init__(self, rocket: Rocket, altimeter: Altimeter, ahrs: AHRS):
        self.rocket = rocket
        self.altimeter = altimeter
        self.ahrs = ahrs

        self.H = np.array([[0.0, 1.0, 0.0, 0.0]])
        self.state = np.zeros(4)
        self.P = np.zeros((4, 4))
        self.R = altimeter.get_variance()

    def time_update(self, dt):
        A = self.rocket.get_A(dt)
        B = self.rocket.get_B(dt)

        Q = self.ahrs.get_variance() * B @ B.T
        self.state = A @ self.state + B @ self.ahrs.get_acceleration()
        self.P = A @ self.P @ A.T + Q

    def measurement_update(self):
        z = self.altimeter.get_altitude()
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.state = self.state + K @ (z - self.H @ self.state)
        self.P = (np.eye(4) - K @ self.H) @ self.P
