import constants.rocket_c as r
import math

class Airbrake:
    def __init__(self):
        self.state = 0

    def set_commanded_state(self, state, dt):
        rate = (state - self.state) / r.servo_tao

        self.state += rate * dt

        if self.state < 0:
            self.state = 0
        if self.state > 1:
            self.state = 1
