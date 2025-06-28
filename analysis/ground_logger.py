import pandas as pd
from datetime import datetime

import constants as c


class GroundLogger:
    def __init__(self, path, names: tuple):
        self.path = f"{path}/{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        self.names = names
        self.frame = pd.DataFrame(columns=names)
        self.primed = [None] * len(names)
        self.new_data = False
        self.index = 0

    def publish(self):
        if self.new_data:
            self.frame.loc[self.index] = self.primed
            self.index += 1
            self.frame.to_csv(self.path)
            self.new_data = False

    def receive(self, data):
        if data is None:
            self.new_data = False
        else:
            split_raw = data.split(sep=c.delimiter)
            for i in range(len(split_raw)):
                self.primed[i] = c.logging_types[i](split_raw[i])
            self.new_data = True

    def get_data(self):
        return self.frame.loc[self.index - 1]

    def new(self):
        return self.new_data
