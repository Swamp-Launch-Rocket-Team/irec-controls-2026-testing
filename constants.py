#all constants use standard si units unless otherwise noted

#enviormental constants
gas_constant = 8.3144598
molar_mass = 0.0289644
density_0 = 1.00
temp_0 = 300.0
gravity = 9.80665
lapse_rate = 0.0065

#rocket constants
post_burn_mass = 22.23 #mass of the rocket post-burn
airbrake_to_altimeter = 2.0 #distance from airbrake to altimeter

#airbrake constants
u = (0.0, 0.5, 1.0)
cd = (0.476, 0.476, 0.476)
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

#sensor constants
ahrs_variance = 0.01
altimeter_variance = 0.01

#simluation constants
velocity_buffer = 1.0