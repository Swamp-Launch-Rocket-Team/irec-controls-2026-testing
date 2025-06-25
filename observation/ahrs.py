import numpy as np


class AHRS:
    def __init__(self, variance):
        self.variance = variance

    def get_variance(self):
        return self.variance

    def get_acceleration(self):
        return np.array([0.0, 0.0])