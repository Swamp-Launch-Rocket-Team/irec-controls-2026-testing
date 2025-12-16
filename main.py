from simulation.sim_rocket import Rocket
import numpy as np
import constants.gnc_c as g
from gnc.gnc import GNC
import constants.rocket_c as c
import matplotlib.pyplot as plt
import constants.atmosphere_c as atm

r = Rocket()
r.add_airbrakes()
r.run_sim()

time_list, deployment_level_list, apogee_list, time_prime_list = [], [], [], []

obs_vars = r.test_flight.get_controller_observed_variables()

t = 0

for time, deployment_level, apogee, dt in obs_vars:
    t += dt
    time_list.append(time)
    deployment_level_list.append(deployment_level)
    apogee_list.append(apogee)
    time_prime_list.append(t)

# Plot deployment level by time
plt.plot(time_list, deployment_level_list)
plt.xlabel("Time (s)")
plt.ylabel("Deployment Level")
plt.title("Deployment Level by Time")
plt.grid()
plt.show()

# Plot drag coefficient by time
plt.plot(time_list, apogee_list)
plt.xlabel("Time (s)")
plt.ylabel("Predicted Apogee (m)")
plt.title("Apogee by Time")
plt.grid()
plt.show()

# start_state = r.get_burnout_state(0)

# r.gnc = GNC(
#     start_state, 
#     r.env.pressure(c.sim_location[2]), 
#     r.env.temperature(c.sim_location[2])
# )

# print(f"density0: {r.gnc.dynamics.atmosphere.get_d0()} kg/m^3")
# print(f"pressure0: {r.env.pressure(c.sim_location[2])} kg/m^3")
# print(f"temp0: {r.env.temperature(c.sim_location[2])} kg/m^3")

# print(f"Initial State: z={start_state[2]} m, vz={start_state[3]} m/s")
# print(f"Dumb Apogee: {start_state[2] + start_state[3]**2/(2*atm.gravity)} m")
# print(f"RocketPy Apogee: {r.get_apogee()[2]} m")
# print(f"Predicted Apogee: {path[-1][2]} m")
