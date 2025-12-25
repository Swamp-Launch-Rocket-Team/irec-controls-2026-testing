import constants.gnc_c as g
import constants.rocket_c as r
import constants.atmosphere_c as a
from gnc.dynamics import DynamicsModel

import numpy as np
from scipy.linalg import solve_continuous_are


class StateController:
    def __init__(self, dynamics_model: DynamicsModel):
        self.dynamics_model = dynamics_model
    
    def get_command_matrix(self, state_0, input_0, nav_state_0):
        J_state, J_input = self.dynamics_model.get_linearized(state_0, input_0, nav_state_0)

        A = J_state.T
        B = J_input.T
        
        S = solve_continuous_are(A, B, g.cQ, g.cR)
        
        K = np.linalg.inv(g.cR) @ B.T @ S

        return K

    def get_control_command(self, true_state, nav_state, time):  
        state_error = true_state - g.target_state
        state_error = np.array([state_error]).T
        
        K = self.get_command_matrix(
            true_state,
            g.target_actuation,
            nav_state
        )

        return g.target_actuation - float(K @ state_error)
