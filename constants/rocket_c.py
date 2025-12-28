import datetime
import math

rocket_radius = 156.9 / 2000
rocket_mass = 18.5
rocket_inertia = (22.52, 22.52, 0.112)
rocket_com = 1.92
rocket_length = 3.54

motor_length = 800 / 1000
motor_radius = 98 / 2000
motor_mass = 4.711
motor_dry_inertia=(
    (1/12)*motor_mass*motor_length**2, 
    (1/12)*motor_mass*motor_length**2, 
    motor_mass*motor_radius**2
)

nozzle_radius = 33 / 2000
nozzle_position = 60 / 1000
grain_number = 4
grain_density = 1815
grain_outer_radius = 89 / 2000
grain_initial_inner_radius = 30 / 2000
grain_initial_height = 142.4 / 1000
grain_separation = 5 / 1000
grains_com_position = 0.42
motor_com_position = 0.37

nose_length = 0.686
nose_kind = "von karman"
nose_position = 0

fins_n = 4
fins_root_chord = 0.254
fins_tip_chord = 0.178
fins_span = 0.152
fins_position = 3.2
fins_cant_angle = 0.0
fins_airfoil = None

tail_top_radius = 156 / 2000
tail_bottom_radius = 126 / 2000
tail_length = 76 / 1000
tail_position = rocket_length - (76 / 1000)

airbrake_actuation_length = 0.0882904 / 2.0
airbrake_actuation_width = 0.06985

sim_location = (31.9996, 102.0757, 847.9536)
sim_date = datetime.date.today() + datetime.timedelta(days=1)

servo_tao = 0.352
servo_time_delay = 0.01
servo_noise = 0.02

rocket_refa = math.pi * rocket_radius**2
rocket_cl_vz = (0.0, 0.0)
rocket_aoa_vel = (0.0100, -0.00002, -0.00003)
rocket_cd_mach = [0.7994, -0.4335, 1.1084, -1.3226, 0.6735] # [0.7994, -0.4335, 1.1084, -1.3226, 0.6735]
rocket_cd_airbrake = (1.0, 0.5)

coast_mass = motor_mass + rocket_mass