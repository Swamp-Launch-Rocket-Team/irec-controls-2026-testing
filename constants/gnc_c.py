import numpy as np
import math

q = 0.0001

Q_cd = np.diag([0, 0, 0, 0, 0.000005, 0.000005])

R = np.array([
    [0.25, 0, 0],
    [0, 1000000, 0],
    [0, 0, 1000000]
])

R_drag = np.array([
    [0.25, 0, 0],
    [0, 0.01, 0],
    [0, 0, 0.01]
])

cQ = np.array([
    [1.0, 0],
    [0, 0]
])

cR = np.array([
    [4.0]
])

cI_domain = 10

target_actuation = 0.5
target_apogee = 2300

target_state = np.array([target_apogee, target_actuation])

dt_guidance = 0.02
guidance_tolerance = 0.1

altimeter_rms = 0.25

ahrs_accel_noise_density = 70E-6
ahrs_accel_bias_stability = 40E-6
ahrs_accel_bandwidth = 230

ahrs_e12_rms = math.radians(0.5)
ahrs_e3_rms = math.radians(2.0)