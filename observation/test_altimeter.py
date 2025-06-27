import pandas as pd

from altimeter import Altimeter
from modeling.rocket import Rocket


class TestAltimeter(Altimeter):
    def __init__(self, dist, rocket: Rocket, noise_path):
        super().__init__(dist)
        self.rocket = rocket
        self.noise = pd.read_csv(noise_path)
        self.t = 0.0

    def get_z(self):
        closest_index = (self.noise['t'] - self.t).abs().idxmin()
        return self.noise.loc[closest_index, 'z']

    def set_t(self, t):
        self.t = t
