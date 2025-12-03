from simulation.sim_rocket import Rocket
import numpy as np
import constants.gnc_c as g
from gnc.gnc import GNC
import constants.rocket_c as c
import matplotlib.pyplot as m
import constants.atmosphere_c as atm

r = Rocket()
r.run_sim()

start_state = r.get_burnout_state(1)

r.gnc = GNC(
    start_state, 
    r.env.pressure_ISA(c.sim_location[2]), 
    r.env.temperature_ISA(c.sim_location[2])
)

path, parameter = r.gnc.path_guidance.generate_path(start_state)

print(f"Initial State: z={start_state[2]} m, vz={start_state[3]} m/s")
print(f"Dumb Apogee: {start_state[2] + start_state[3]**2/(2*atm.gravity)} m")
print(f"RocketPy Apogee: {r.get_apogee()[2]} m")
print(f"Predicted Apogee: {path[-1][2]} m")
