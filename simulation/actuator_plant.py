import constants.rocket_c as r
import math

class Airbrake:
    def __init__(self):
        self.state = 0

    def set_commanded_state(self, state, dt):
        if state < 0:
            state = 0
        if state > 1:
            state = 1

        rate = (state - self.state) / r.servo_tao
        self.state += rate * dt
