import numpy as np
from scipy.interpolate import interp1d  

from dynamics import DynamicsModel
import constants.gnc_c as g

class PathGuidance:
    def __init__(self, dynamics_model: DynamicsModel):
        self.dynamics_model = dynamics_model
    
    def get_apogee_error(self, state_0):
        parameter = self.generate_path(state_0)[1]
        
        return parameter[-1] - g.target_apogee

    def generate_path(self, state_0):
        path = np.array([state_0])
        parameter = np.array([state_0[2]])
        state_i = state_0.copy()

        while parameter[-1] > 0:
            # RK4 integration
            k1 = self.dynamics_model.get_state_derivative(state_i, g.target_actuation)
            k2 = self.dynamics_model.get_state_derivative(state_i + 0.5*g.dt_guidance*k1, g.target_actuation)
            k3 = self.dynamics_model.get_state_derivative(state_i + 0.5*g.dt_guidance*k2, g.target_actuation)
            k4 = self.dynamics_model.get_state_derivative(state_i + g.dt_guidance*k3, g.target_actuation)
            
            state_i = state_i + (g.dt_guidance/6.0)*(k1 + 2*k2 + 2*k3 + k4)
            
            path = np.append(path, [state_i], axis=0)
            parameter = np.append(parameter, [state_i[2]])
        
        return path, parameter
       
        
    def get_new_path(self, state_0):
        # Two initial guesses (perturb the altitude)
        state_0_copy = state_0.copy()
        state_1 = state_0.copy()
        state_1[2] += 100  # perturb altitude by 100 units
        
        error_0 = self.get_apogee_error(state_0_copy)
        error_1 = self.get_apogee_error(state_1)
        
        max_iterations = 20
        tolerance = 1.0
        
        for i in range(max_iterations):
            if abs(error_1) < tolerance:
                break
                
            # Secant formula: x_new = x1 - f1 * (x1 - x0) / (f1 - f0)
            altitude_new = state_1[2] - error_1 * (state_1[2] - state_0_copy[2]) / (error_1 - error_0)
            
            # Update for next iteration
            state_0_copy[2] = state_1[2]
            error_0 = error_1
            state_1[2] = altitude_new
            error_1 = self.get_apogee_error(state_1)
        
        path, parameter = self.generate_path(state_1)
        return interp1d(parameter, path.T, kind="linear", axis=1)
