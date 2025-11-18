import numpy as np
from scipy.interpolate import interp1d  

from dynamics import DynamicsModel
import constants.gnc_c as g

class PathGuidance:
    def __init__(self, dynamics_model):
        self.dynamics_model = dynamics_model
    
    def get_apogee_error(self, state_0):
        state_i = state_0
        while state_i[3] > 0:
            state_i += self.dynamics_model.get_state_derivative(state_i, g.target_actuation) * g.dt_guidance
        
        return state_i[2] - g.target_apogee

    def generate_path(self, state_0):
        path = np.array([state_0])
        parameter = np.array([state_0[2]])

        while parameter[-1] > 0:
            state_i = path[-1] + self.dynamics_model.get_state_derivative(state_i, g.target_actuation) * g.dt_guidance
            path = np.append(path, [state_i], axis=0)
            parameter = np.append(parameter, [state_i[2]])
        
        return path, parameter, input
       
        
    def get_new_path(self, state_0):
        error = 1000

        #automatic convergence interestingly
        while error > 1:
            error = self.get_apogee_error(state_0)
            state_0 -= error * np.array([0, 0, 1, 0, 0])
        
        path, parameter = self.generate_path(state_0)

        return interp1d(parameter, path, kind="linear")
