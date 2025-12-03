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
        self.accelerometer_rms = self.accelerometer_noise_density / np.sqrt(self.accelerometer_bandwidth)
    
    def quaternion_to_euler(self, e0, e1, e2, e3):
        # Roll (rotation about X-axis)
        roll = np.arctan2(2*(e0*e1 + e2*e3), 1 - 2*(e1**2 + e2**2))
        
        # Pitch (rotation about Y-axis)
        sin_pitch = 2*(e0*e2 - e3*e1)
        sin_pitch = np.clip(sin_pitch, -1.0, 1.0)  # Clamp for numerical stability
        pitch = np.arcsin(sin_pitch)
        
        # Yaw (rotation about Z-axis)
        yaw = np.arctan2(2*(e0*e3 + e1*e2), 1 - 2*(e2**2 + e3**2))
        
        return np.array([roll, abs(pitch), yaw])
    
    def get_acceleration(self, v1, v0, dt):
        return (v1 - v0) / dt

    def get_noisy_acceleration(self, a_clean):
        return a_clean + np.random.normal(loc=0, scale=self.accelerometer_rms, size=3) + self.accelerometer_bias

    def get_noisy_euler(self, euler_clean):
        e1_noise, e2_noise = np.random.normal(loc=0, scale=self.e12_rms, size=2)
        e3_noise = np.random.normal(loc=0, scale=self.e3_rms)
        return euler_clean + np.array([e1_noise, e2_noise, e3_noise])
