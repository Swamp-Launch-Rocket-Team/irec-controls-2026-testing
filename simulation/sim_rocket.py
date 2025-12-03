import rocketpy
import numpy as np
import scipy
import datetime

import constants.rocket_c as r
from gnc.gnc import GNC
from simulation.noisy_ahrs import Noisy_AHRS
from simulation.noisy_altimeter import Noisy_Altimeter
from simulation.actuator_plant import Airbrake

class Rocket:
    def __init__(self):
        self.env = rocketpy.Environment(
            latitude=r.sim_location[0], longitude=r.sim_location[1], elevation=r.sim_location[2]
        )

        tomorrow = datetime.date.today() + datetime.timedelta(days=1)
        self.env.set_date((tomorrow.year, tomorrow.month, tomorrow.day, 12))
        self.env.set_atmospheric_model(type="Forecast", file="GFS")

        self.motor = rocketpy.SolidMotor(
            thrust_source="simulation/AeroTech_M2500T.eng",
            dry_mass=r.motor_mass,
            dry_inertia=r.motor_dry_inertia,
            nozzle_radius=r.motor_radius,
            grain_number=r.grain_number,
            grain_density=r.grain_density,
            grain_outer_radius=r.grain_outer_radius,
            grain_initial_inner_radius=r.grain_initial_inner_radius,
            grain_initial_height=r.grain_initial_height,
            grain_separation=r.grain_separation,
            grains_center_of_mass_position=r.grains_com_position,
            center_of_dry_mass_position=r.motor_com_position
        )

        self.dino = rocketpy.Rocket(
            radius= r.rocket_radius,
            mass=r.rocket_mass,
            inertia=r.rocket_inertia,
            power_off_drag="constants/CD Power Off.CSV",
            power_on_drag="constants/CD Power On.CSV",
            center_of_mass_without_motor=r.rocket_com,
            coordinate_system_orientation="nose_to_tail"
        )

        self.dino.add_motor(self.motor, r.rocket_length)

        self.dino.add_trapezoidal_fins(
            n=r.fins_n,
            root_chord=r.fins_root_chord,
            tip_chord=r.fins_tip_chord,
            span=r.fins_span,
            position=r.fins_position,
            cant_angle=r.fins_cant_angle,
            airfoil=r.fins_airfoil
        )

        self.dino.add_nose(
            length=r.nose_length, kind=r.nose_kind, position=r.nose_position
        )

        self.dino.add_tail(
            top_radius=r.tail_top_radius, 
            bottom_radius=r.tail_bottom_radius, 
            length=r.tail_length, 
            position=r.tail_position
        )

        self.test_flight = None

        self.ahrs = Noisy_AHRS()
        self.altimeter = Noisy_Altimeter()
        self.actuator = Airbrake()

        self.gnc = None
        self.input = 0

        self.time = 0
        self.loop_number = 0

        self.g_frequency = 100
        self.c_frequency = 4
        self.nu_frequency = 5
        self.np_frequency = 1


    def add_airbrakes(self):
        self.air_brakes = self.dino.add_air_brakes(
            drag_coefficient_curve="constants\CD Airbrake.csv",
            controller_function=self.update_current_actuation,
            sampling_rate=200,
            reference_area=None,
            clamp=True,
            override_rocket_drag=False,
            name="Air Brakes",
        )
    
    def call_gnc_pre_burnout(self, y, accel, dt, p0, t0):
        if self.loop_number == 0:
            self.gnc = GNC(
                np.array([0, 0, 0, 0, 0]),
                p0, t0
            )
        if self.loop_number % self.np_frequency == 0:
            self.gnc.nav_propegate(dt, accel)
        if self.loop_number % self.nu_frequency == 0:
            self.gnc.nav_update(y)

    def call_gnc_post_burnout(self, y, accel, dt):
        if self.loop_number % self.np_frequency == 0:
            self.gnc.nav_propegate(dt, accel)
        if self.loop_number % self.nu_frequency == 0:
            self.gnc.nav_update(y)
        if self.loop_number % self.g_frequency == 0:
            self.gnc.recompute_guidance()
        if self.loop_number % self.c_frequency == 0 and self.gnc.path is not None:
            self.gnc.control_update()

    def update_current_actuation(self, time, cycle_frequency, state, state_history, observed_variables, brakes):
        burn_out = time > self.motor.burn_out_time + 0.25
        dt = 1.0 / cycle_frequency
        x, y, z, v_x, v_y, v_z, e0, e1, e2, e3, w_x, w_y, w_z = state
        z = z - self.env.elevation
        
        euler_xyz = self.ahrs.quaternion_to_euler(e0, e1, e2, e3)
        noisy_euler_xyz = self.ahrs.get_noisy_euler(euler_xyz)

        v1 = np.array([v_x, v_y, v_z])
        v0 = np.array(state_history[-1][3:6])

        if self.loop_number > 1:
            v0 = np.array(state_history[-2][4:7])
            dt = time - np.array(state_history[-2][0])
        
        a = self.ahrs.get_acceleration(v1, v0, dt)
        noisy_a = self.ahrs.get_noisy_acceleration(a)
        
        v_xy = np.array([v_x, v_y])
        a_xy = np.dot(v_xy, np.array([noisy_a[0], noisy_a[1]])) / (np.linalg.norm(v_xy) + 0.00001)

        z_noisy = self.altimeter.get_noisy_altitude(z)

        sensors = np.array([z_noisy, noisy_euler_xyz[1], self.actuator.state])
        

        if burn_out:
            self.call_gnc_post_burnout(sensors, np.array([a_xy, a[2]]), dt)
        else:
            self.call_gnc_pre_burnout(
                sensors, 
                np.array([a_xy, a[2]]),
                dt,
                self.env.pressure_ISA(r.sim_location[2]), 
                self.env.temperature_ISA(r.sim_location[2]),
            )

        self.actuator.set_commanded_state(self.gnc.input, dt)
        brakes.deployment_level = self.actuator.state

        if self.loop_number > 1:
            print(time)
            print(z)
            print(a[2])
            print(a)
            print(state)
            print(state_history[-2])
            print(sensors)
            print(self.gnc.compass.get_optimal_state())
            print(self.actuator.state)

        self.loop_number += 1

    def run_sim(self):
        self.test_flight = rocketpy.Flight(
            rocket=self.dino, environment=self.env, rail_length=5.2, inclination=85, heading=0
        )

    def plot_results(self):
        self.test_flight.plots.linear_kinematics_data()
        self.test_flight.plots.trajectory_3d()
    
    def get_apogee(self):
        state = self.test_flight.apogee_state
        state[2] = state[2] - self.env.elevation

        return state
    
    def get_burnout_state(self, t_post_burnout=0.25):
        state = self.test_flight.get_solution_at_time(self.motor.burn_out_time + t_post_burnout)
        return np.array([
            0,
            (state[4]**2 + state[5]**2)**0.5,
            state[3] - self.env.elevation,
            state[6],
            0
        ])