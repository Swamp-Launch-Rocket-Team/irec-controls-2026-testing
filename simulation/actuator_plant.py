import constants.rocket_c as r
import math
import numpy as np

class Airbrake:
    def __init__(self):
        self.state = 0
        self.rate = 0

    def set_commanded_state(self, state, dt):
        if state < 0:
            state = 0
        if state > 1:
            state = 1

        self.rate = (state - self.state) / 0.352
        self.state += self.rate * dt
    
    def get_state(self):
        base_state = self.state
        base_state += np.random.normal(loc=0, scale=r.servo_noise)

        return base_state
    
    
