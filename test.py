import numpy as np
import scipy
import matplotlib.pyplot as plt
import matplotlib
import math

from matplotlib.pyplot import xlabel

import drag_model, dynamics_model

drag = drag_model.DragModel()
rocket = dynamics_model.DynamicsModel(drag)
drag_constant = drag.get_drag_constant(0)

print(rocket.project_apogee(
    np.array([
        178.894,
        1235.71,
        math.sqrt(40.024**2 + 249.966**2),
        math.atan2(249.966, 40.024)
    ]),
    np.array([drag_constant]),
    0.01
))

target = 1500.0
angle = 0.0
veloctiy_range = 100
drag_range = drag.get_drag_constant(1) * 1.5

velocty_increments = 10
drag_increments = 10

delta_t = 0.01
height = 0

trajectories = []

for j in range(drag_increments):
    const_drag_trajectories = []
    for i in range(velocty_increments):
        const_drag_trajectories.append(rocket.project_trajectory_height(
            np.array([
                0.0,
                target,
                i * veloctiy_range / velocty_increments,
                angle
            ]),
            np.array([j * drag_range / drag_increments]),
            -delta_t,
            height
        ))
    trajectories.append(const_drag_trajectories)

x = np.array([])
z = np.array([])
v = np.array([])
theta = np.array([])

for traj in trajectories[5]:
    for state in traj:
        x = np.append(x, state[1][0])
        z = np.append(z, state[1][1])
        v = np.append(v, state[1][2])
        theta = np.append(theta, state[1][3])

print("Trajectoies Computed.")
v_grid = np.linspace(min(v), max(v), 50)
theta_grid = np.linspace(min(theta), max(theta), 50)
V, T = np.meshgrid(v_grid, theta_grid)



print("Meshgrid Created.")
points = np.array([v, theta]).T
Z = scipy.interpolate.griddata(points, z, (V, T), method='cubic')

matplotlib.use('TkAgg')
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

ax.plot_surface(V, T, Z)
ax.set_title('Trajectory Info')

ax.set_xlabel('Velocity (m/s)')
ax.set_ylabel('Theta (Radians)')
ax.set_zlabel('Z (meters)')
plt.show()