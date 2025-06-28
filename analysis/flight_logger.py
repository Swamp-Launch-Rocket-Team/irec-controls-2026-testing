import pandas as pd
from datetime import datetime

import constants as c


class FlightLogger:
    def __init__(self, path, names: tuple):
        self.path = f"{path}/{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
        self.names = names
        self.frame = pd.DataFrame(columns=names)
        self.primed = [None] * len(names)
        self.index = 0

    def prime(self, types, data):
        for i in range(len(types)):
            primed_index = self.names.index(types[i])
            self.primed[primed_index] = data[i]

    def publish(self):
        self.frame.loc[self.index] = self.primed
        self.index += 1

    def transmit(self):
        raw_data = ""
        for data in self.primed:
            raw_data += str(data) + c.delimiter

        return raw_data[:-1]

    def reset(self):
        self.primed = [None] * len(self.names)

    def save(self):
        self.frame.to_csv(self.path)
