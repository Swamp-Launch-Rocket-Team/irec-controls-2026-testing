import rocketpy
import numpy as np
import scipy
import datetime

import constants.rocket_c as r
import constants.gnc_c as g
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
        self.env.set_atmospheric_model(type="standard_atmosphere")

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

        self.c_frequency = 5
        self.nu_frequency = 2
        self.np_frequency = 1


    def add_airbrakes(self):
        self.air_brakes = self.dino.add_air_brakes(
            drag_coefficient_curve="constants\CD Airbrake.csv",
            controller_function=self.update_current_actuation,
            sampling_rate=100,
            reference_area=None,
            clamp=True,
            override_rocket_drag=False,
            name="Air Brakes",
        )
    
    def call_gnc_pre_burnout(self, nav_output, a_output, nav_input, dt, p0, t0):
        if self.loop_number == 0:
            self.gnc = GNC(p0, t0)
        if self.loop_number % self.np_frequency == 0:
            self.gnc.nav_propegate(dt, nav_input)
        if self.loop_number % self.nu_frequency == 0:
            self.gnc.nav_update(nav_output)
            self.gnc.actuator_update(a_output)

    def call_gnc_post_burnout(self, nav_output, drag_output, a_output, nav_input, dt):
        if self.loop_number % self.np_frequency == 0:
            self.gnc.drag_propegate(dt)
            self.gnc.nav_propegate(dt, nav_input)
        if self.loop_number % self.nu_frequency == 0:
            self.gnc.drag_update(drag_output)
            self.gnc.nav_update(nav_output)
            self.gnc.actuator_update(a_output)
        if self.loop_number % self.c_frequency == 0:
            self.gnc.control_update()

    def update_current_actuation(self, time, cycle_frequency, state, state_history, observed_variables, brakes):
        burn_out = time > self.motor.burn_out_time + 0.25
        dt = 1.0 / cycle_frequency
        dt_a = 1.0 / cycle_frequency

        x, y, z, v_x, v_y, v_z, e0, e1, e2, e3, w_x, w_y, w_z = state
        z = z - self.env.elevation
        
        euler_xyz = self.ahrs.quaternion_to_euler(e0, e1, e2, e3)
        noisy_euler_xyz = self.ahrs.get_noisy_euler(euler_xyz)

        v1 = np.array([v_x, v_y, v_z])
        v0 = np.array(state_history[-1][3:6])

        if self.loop_number > 1:
            v0 = np.array(state_history[-2][4:7])
            dt_a = time - state_history[-2][0]
        
        a = self.ahrs.get_acceleration(v1, v0, dt_a)
        noisy_a = self.ahrs.get_noisy_acceleration(a)
        
        v_xy = np.array([v_x, v_y])
        a_xy = np.dot(v_xy, np.array([noisy_a[0], noisy_a[1]])) / (np.linalg.norm(v_xy) + 0.000001)

        z_noisy = self.altimeter.get_noisy_altitude(z)

        nav_output = np.array([z_noisy])
        nav_input = np.array([a_xy, a[2]])
        drag_output = nav_input.copy()

        if burn_out:
            self.call_gnc_post_burnout(
                nav_output,
                drag_output,
                self.actuator.get_state(), 
                nav_input,
                dt
            )
        else:
            self.call_gnc_pre_burnout(
                nav_output, 
                self.actuator.get_state(),
                nav_input,
                dt,
                self.env.pressure_ISA(r.sim_location[2]), 
                self.env.temperature_ISA(r.sim_location[2]),
            )

        self.actuator.set_commanded_state(self.gnc.input, dt)
        brakes.deployment_level = self.actuator.state
        
        x_nav = self.gnc.compass.get_optimal_nav_state()
        x_h = self.gnc.compass.get_optimal_state()
        x_drag = self.gnc.compass.get_optimal_drag_state()


        if self.loop_number > 1 and self.loop_number % 50 == 0:
            print(f"---   iteration {self.loop_number}   ---")
            print(f"time: {time}\ndt: {dt}\nz: {z}\nvz: {v_z}\napogee: {x_h[0]}")
            print(f"predicted z: {x_nav[2]}\npredicted vz: {x_nav[3]}")
            print(f"axy: {a_xy}\naz: {a[2]}")
            print(f"input: {self.gnc.input}\nactuator: {self.actuator.get_state()}")
            print(f"cD_1: {x_drag[0]}\ncD_2: {x_drag[1]}\ncL_1: {x_drag[2]}\ncL_2: {x_drag[3]}")
            print(f"P_c1: {self.gnc.compass.drag_P[0][0]}\nP_c2: {self.gnc.compass.drag_P[1][1]}")
            print(f"P_c3: {self.gnc.compass.drag_P[2][2]}\nP_c3: {self.gnc.compass.drag_P[3][3]}\n")

        self.loop_number += 1
        return (
            time,
            self.actuator.state,
            max(min(x_h[0] - g.target_state[0], 150), -150),
            dt
        )

    def run_sim(self):
        self.test_flight = rocketpy.Flight(
            rocket=self.dino, environment=self.env, rail_length=5.2, inclination=85, heading=0, terminate_on_apogee=True
        )
        self.loop_number = 0

    def plot_results(self):
        self.test_flight.plots.linear_kinematics_data()
        self.test_flight.plots.trajectory_3d()
    
    def get_apogee(self):
        state = self.test_flight.apogee_state
        state[2] = state[2] - self.env.elevation

        return state[2]
    
    def get_burnout_state(self, t_post_burnout=0.25):
        state = self.test_flight.get_solution_at_time(self.motor.burn_out_time + t_post_burnout)
        return np.array([
            0,
            (state[4]**2 + state[5]**2)**0.5,
            state[3] - self.env.elevation,
            state[6]
        ])