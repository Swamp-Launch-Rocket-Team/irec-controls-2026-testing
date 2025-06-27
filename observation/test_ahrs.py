from numpy.random import normal
import numpy as np

from ahrs import AHRS
import constants as c
from modeling.rocket import Rocket


class TestAHRS(AHRS):
    def test_ahrs(self, translational_dist, angular_dist, rocket: Rocket):
        super().__init__(translational_dist, angular_dist)
        self.bias_t_x = normal(0, c.ahrs_test_bias_t)
        self.bias_t_z = normal(0, c.ahrs_test_bias_t)
        self.bias_r = normal(0, c.ahrs_test_bias_r)
        self.rocket = rocket

    def get_acceleration(self):
        mean = self.rocket.state_acceleration(self.rocket.get_u())
        bias = np.array([self.bias_t_x, self.bias_t_z])
        noise = np.array([normal(0, self.dist_t), normal(0, self.dist_t)])
        return mean + bias + noise

    def get_pitch_angle(self):
        return 0.0
