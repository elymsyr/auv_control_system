import numpy as np
from model import Model

class PID():
    def __init__(self):
        self.model = Model()
        self.N1i = [2.0, 2.0, 1.0, 2.0]
        self.sigma1i = [1.0, 1.274, 0.5, 16.0]