import numpy as np
from gnc.control import StateController
from gnc.navigation import KalmanFilter
from gnc.atmosphere import AtmosphereModel
from gnc.dynamics import DynamicsModel

class GNC:
    def __init__(self, p0, t0):
        self.dynamics = DynamicsModel(AtmosphereModel(p0, t0))
        self.compass = KalmanFilter(self.dynamics)
        self.controller = StateController(self.dynamics)
        self.input = 0

    def control_update(self):
        x_h = self.compass.get_optimal_state()
        x_nav = self.compass.get_optimal_nav_state()
        x_drag = self.compass.get_optimal_drag_state()

        self.input = self.controller.get_control_command(x_h, x_nav, x_drag)

    def actuator_update(self, a_output):
        self.compass.update_actuator(a_output)

    def nav_update(self, nav_output):
        self.compass.update(nav_output)
    
    def drag_update(self, drag_output):
        self.compass.update_drag(drag_output)
    
    def nav_propegate(self, dt, nav_input):
        self.compass.predict(dt, nav_input)
    
    def drag_propegate(self, dt):
        self.compass.predict_drag(dt)
