import numpy as np
import math

q = 0.0001

R = np.array([
    [0.25, 0],
    [0, 0.05]
])

cQ = np.array([
    [0.25, 0],
    [0, 0]
])

cR = np.array([
    [1.0]
])

target_actuation = 0.5
target_apogee = 2400

target_state = np.array([target_apogee, target_actuation])

dt_guidance = 0.01
guidance_tolerance = 0.1

altimeter_rms = 0.25

ahrs_accel_noise_density = 70E-9
ahrs_accel_bias_stability = 40E-9
ahrs_accel_bandwidth = 230

ahrs_e12_rms = math.radians(0.5)
ahrs_e3_rms = math.radians(2.0)