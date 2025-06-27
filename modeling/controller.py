from scipy.interpolate import LinearNDInterpolator

import constants as c
from airbrake import Airbrake
from modeling.smc import SMC
from rocket import Rocket

import numpy as np
from scipy import interpolate


class RocketController:
    def __init__(self, target_apogee, environment, rocket, altitude_floor=0.0, target_input=0.5):
        self.target_input = target_input
        self.target_apogee = target_apogee
        self.altitude_floor = altitude_floor
        self.environment = environment
        self.rocket: Rocket = rocket
        self.surface: LinearNDInterpolator = self.generate_sliding_surface()
        self.controller = self.generate_controller()

    def generate_sliding_surface(self):
        points = [] # list of <z, x-dot>
        values = [] # list of <z-dot>

        initial_v = 0.0
        while initial_v <= c.max_final_v:
            new_points, new_values = self.generate_target_trajectory(initial_v)
            points.extend(new_points)
            values.extend(new_values)
            initial_v += c.max_final_v / c.v_resolution

        z_grid, x_dot_grid = np.meshgrid(
            np.linspace(self.altitude_floor, self.target_apogee, c.surface_resolution[0]),
            np.linspace(0.0, 0.5*np.pi, c.surface_resolution[1])
        )

        points_grid = np.array([z_grid.flatten(), x_dot_grid.flatten()]).T

        grid_data = interpolate.griddata(
            points, np.array(values), points_grid, method="cubic", fill_value=0.0
        )

        return interpolate.LinearNDInterpolator(
            points_grid,
            grid_data,
            fill_value=0.0
        )

    def generate_target_trajectory(self, initial_v):
        rocket = Rocket(self.environment, Airbrake(c.max_rate))
        rocket.set_state(np.array([0, self.target_apogee, initial_v, 0]))
        points = [(rocket.state[1], rocket.state[2])]
        values = [rocket.state[3]]
        t_accum = 0.0

        while rocket.state[1] > self.altitude_floor:
            new_state = rocket.project_next_state(self.target_input, -c.dt_dynamics)
            rocket.set_state(new_state)
            t_accum += c.dt_dynamics

            if t_accum > c.dt_sample:
                points.append((rocket.state[1], rocket.state[2]))
                values.append(rocket.state[3])
                t_accum = 0.0

        return points, values

    def generate_controller(self, type="smc", flavor="saturation"):
        if type == "smc":
            return SMC(self.target_apogee, self.target_input, self.surface, c.convergence_time, flavor)
        else:
            raise ValueError(f"Invalid controller type: {type}")

    def get_output(self):
        """
        :return: normalized output between 0.0 and 1.0 (airbrake command)
        """
        self.controller.get_output(self.rocket.state, self.rocket.get_authority_rate())
