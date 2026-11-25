import numpy as np

class Goal:
    def __init__(self, x : np.ndarray, value : int, index: int):
        self.pos = x
        self.reward = value
        self.index = index