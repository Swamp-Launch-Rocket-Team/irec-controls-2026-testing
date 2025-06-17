import math

import numpy as np
import drag_model
import constants as const

class DynamicsModel:
    def __init__(self, drag: drag_model.DragModel):
        self.state = np.array([0, 0, 0, 0]) # (x, z, v, theta)
        self.input = np.array([0]) # drag const -> 0.5 * Cd * A * density0
        self.drag = drag

    def state_derivative(self, state: np.ndarray, input: np.ndarray) -> np.ndarray:
        return np.array([
            state[2] * np.cos(state[3]),
            state[2] * np.sin(state[3]),
            -drag_model.get_drag(input[0], state[2], state[1]) - const.gravity * np.sin(state[3]),
            -(const.gravity * np.cos(state[3])) / math.sqrt(math.pow(state[2], 2) + math.pow(const.velocity_buffer, 2)) #0.001 to prevent divide by zero error
        ])

    def get_next_state(self, state: np.ndarray, input: np.ndarray, delta_t) -> np.ndarray:
        state_derivative = self.state_derivative(state, input)
        return state + state_derivative * delta_t

    def project_trajectory_time(self, state: np.ndarray, input: np.ndarray, delta_t, final_t) -> list:
        t: float = 0
        states: list = [(0, state, input)]

        while abs(t) <= abs(final_t):
            t += delta_t
            states.append((t, self.get_next_state(states[-1][1], input, delta_t), input))

        return states

    def project_trajectory_height(self, state: np.ndarray, input: np.ndarray, delta_t, final_height) -> list:
        t: float = 0
        states: list = [(0, state, input)]

        while states[-1][1][1] > final_height:
            t += delta_t
            states.append((t, self.get_next_state(states[-1][1], input, delta_t), input))

        return states

    def project_apogee(self, state: np.ndarray, input: np.ndarray, delta_t):
        t = 0
        current_state = np.array([0, 0, 0, 0])
        new_state = state

        while new_state[1] > current_state[1]:
            t += delta_t
            current_state = new_state
            new_state = self.get_next_state(current_state, input, delta_t)

        return current_state

