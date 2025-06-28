#all constants use standard si units unless otherwise noted

#enviormental constants
gas_constant = 8.3144598
molar_mass = 0.0289644
density_0 = 1.00
temp_0 = 300.0
gravity = 9.80665
lapse_rate = 0.0065
gamma = 1.4

#rocket constants
post_burn_mass = 22.23 #mass of the rocket post-burn

#airbrake constants
u = (0.0, 0.5, 1.0)
mach = (0.0, 0.5, 1.0)

cd = (0.476, 0.476, 0.476)
cd_reduction = (1.0, 0.9, 0.8)

a = (0.020, 0.025, 0.030)
pwm = (0, 120, 240)
max_rate = 10.9

#controller constants
v_resolution = 50
max_final_v = 40.0
target_apogee = 1000.0
dt_dynamics = 0.0001
dt_sample = 0.01
surface_resolution = (50, 50)
convergence_time = 3

#sensor constants
ahrs_variance_t = 0.01
ahrs_variance_r = 0.01
altimeter_variance = 0.01

#simluation constants
velocity_buffer = 1.0

#test constants
ahrs_test_bias_t = 0.0001
ahrs_test_variance_t = 0.0001
ahrs_test_bias_r = 0.0001
ahrs_test_variance_r = 0.0001

#data analysis constants
logging_values = ("t", "x", "z", "vx", "vz", "U", "ax", "az", "theta", "altimeter")
logging_types = (float, float, float, float, float, float, float, float, float, float)
delimiter = "|"