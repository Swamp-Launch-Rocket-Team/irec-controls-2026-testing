import numpy as np
from control import StateController
from guidance import PathGuidance
from navigation import KalmanFilter
from atmosphere import AtmosphereModel
from dynamics import DynamicsModel

class GNC:
    def __init__(self, x0, p0, t0):
        self.dynamics = DynamicsModel(AtmosphereModel(p0, t0))
        self.compass = KalmanFilter(self.dynamics)
        self.path_guidance = PathGuidance(self.dynamics, np.array([0, 0, 0, 0, 0]))
        self.controller = StateController(self.dynamics)
        self.path = None

    def recompute_guidance(self):
        x_h = self.compass.get_optimal_state()
        self.path = self.path_guidance.get_new_path(x_h)

    def get_command(self):
        if self.path is None:
            return None
        
        x_h = self.compass.get_optimal_state()
        x_target = self.path(x_h[3])

        return self.controller.get_control_command(x_h, x_target)
    
    def nav_update(self, y_i):
        self.compass.update(y_i)
    
    def nav_propegate(self, dt, input):
        self.compass.predict(dt, input)
