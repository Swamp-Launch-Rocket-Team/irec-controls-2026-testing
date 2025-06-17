import numpy

class SlidingModeController:

    def __init__(self, target_input, target_apogee, environment):
        self.target_input = target_input
        self.target_apogee = target_apogee
