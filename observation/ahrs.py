import numpy as np

class AHRS:
    def __init__(self, translational_dist, angular_dist):
        self.dist_t = translational_dist
        self.dist_r = angular_dist

    def get_variance_t(self):
        return self.dist_t**2

    def get_variance_r(self):
        return self.dist_r**2

    def get_acceleration(self):
        return np.array([0.0, 0.0])

    def get_pitch_angle(self):
        return 0.0