import numpy as np
from scipy.interpolate import interp1d  

from gnc.dynamics import DynamicsModel
import constants.gnc_c as g

class PathGuidance:
    def __init__(self, dynamics_model: DynamicsModel):
        self.dynamics_model = dynamics_model
    
    def get_apogee_error(self, state_0):
        path = self.generate_path(state_0)[0]
        
        return path[-1][2] - g.target_apogee

    def generate_path(self, state_0):
        path = np.array([state_0])
        parameter = np.array([state_0[3]])
        state_i = state_0.copy()

        while parameter[-1] > 0:
            # RK4 integration
            k1 = self.dynamics_model.get_state_derivative(state_i, 0)
            state_i = state_i + g.dt_guidance*k1
            
            path = np.append(path, [state_i], axis=0)
            parameter = np.append(parameter, [state_i[3]])
        
        return path, parameter
       
        
    def get_new_path(self, state_0):
        error = 1000

        #automatic convergence interestingly
        while error > 1:
            error = self.get_apogee_error(state_0)
            state_0 -= error * np.array([0, 0, 1, 0, 0])
        
        path, parameter = self.generate_path(state_0)

        return interp1d(parameter, path.T, kind="linear", bounds_error=False, fill_value='extrapolate', axis=1)
