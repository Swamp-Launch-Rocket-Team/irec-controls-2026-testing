import math
import scipy
import numpy as np
import constants.gnc_c as g

class Noisy_AHRS:
    def __init__(self):
        self.accelerometer_noise_density = g.ahrs_accel_noise_density
        self.accelerometer_bias_stability = g.ahrs_accel_bias_stability
        self.accelerometer_bandwidth = g.ahrs_accel_bandwidth
        
        self.e12_rms = g.ahrs_e12_rms
        self.e3_rms = g.ahrs_e3_rms

        self.accelerometer_bias = np.random.normal(loc=0, scale=self.accelerometer_bias_stability, size=3)
        self.accelerometer_rms = np.random.normal(
            loc=0, 
            scale=self.accelerometer_noise_density / np.sqrt(self.accelerometer_bandwidth), 
            size=3
        )

    def get_noisy_acceleration(self, a_clean):
        return a_clean + np.random.normal(loc=0, scale=self.accelerometer_rms, size=3) + self.accelerometer_bias

    def get_noisy_euler(self, euler_clean):
        e1_noise, e2_noise = np.random.normal(loc=0, scale=self.e12_rms, size=2)
        e3_noise = np.random.normal(loc=0, scale=self.e3_rms)
        return euler_clean + np.array([e1_noise, e2_noise, e3_noise])
