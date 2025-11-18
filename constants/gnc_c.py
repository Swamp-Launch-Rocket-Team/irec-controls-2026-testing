import numpy as np
import math

Q = np.array([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0]
])

R = np.array([
    [1]
])

target_actuation = 0.5
target_apogee = 3048

dt_guidance = 0.02
guidance_tolerance = 0.1

altimeter_rms = 0.25

ahrs_accel_noise_density = 70E-9
ahrs_accel_bias_stability = 40E-9
ahrs_accel_bandwidth = 230

ahrs_e12_rms = math.radians(0.5)
ahrs_e3_rms = math.radians(2.0)