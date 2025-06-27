class Altimeter:
    def __init__(self, dist):
        self.dist = dist

    def get_variance(self):
        return self.dist**2

    def get_z(self):
        return 0.0