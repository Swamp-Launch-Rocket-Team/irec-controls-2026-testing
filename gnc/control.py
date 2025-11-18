import constants.gnc_c as g
import constants.rocket_c as r
import constants.atmosphere_c as a
from dynamics import DynamicsModel

import numpy as np
from scipy.linalg import solve_continuous_are


class StateController:
    def __init__(self, dynamics_model: DynamicsModel):
        self.dynamics_model = dynamics_model
    
    def get_command_matrix(self, guidance_state, guidance_input):
        J_state, J_input = self.dynamics_model.get_linearized(guidance_state, guidance_input)
        A = J_state.transpose()
        B = J_input.transpose()
        
        S = solve_continuous_are(A, B, g.Q, g.R)
        K = np.linalg.inv(g.R) @ B.T @ S

        return K

    def get_control_command(self, true_state, guidance_state, guidance_input=g.target_actuation):
        K = self.get_command_matrix(guidance_state, guidance_input)
        return guidance_input - K @ (true_state - guidance_state)
