import math
import scipy
import numpy as np
import constants.gnc_c as g

class Noisy_Altimeter:
    def __init__(self):
        self.z_rms = g.altimeter_rms
        
    def get_noisy_altitude(self, z_clean):
        return z_clean + np.random.normal(loc=0, scale=self.z_rms)
        