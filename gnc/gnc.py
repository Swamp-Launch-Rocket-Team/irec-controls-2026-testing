import numpy as np
from gnc.control import StateController
from gnc.navigation import KalmanFilter
from gnc.atmosphere import AtmosphereModel
from gnc.dynamics import DynamicsModel

class GNC:
    def __init__(self, x0, p0, t0):
        self.dynamics = DynamicsModel(AtmosphereModel(p0, t0))
        self.compass = KalmanFilter(self.dynamics)
        self.controller = StateController(self.dynamics)
        self.input = 0

    def control_update(self):
        x_h = self.compass.get_optimal_state()
        x_nav = self.compass.get_optimal_nav_state()
        self.input = self.controller.get_control_command(x_h, x_nav)
    
    def nav_update(self, y_i):
        self.compass.update(y_i)
    
    def actuator_update(self, U_actual):
        self.compass.update_actuator(U_actual)
    
    def nav_propegate(self, dt, accel):
        self.compass.predict(dt, accel)
