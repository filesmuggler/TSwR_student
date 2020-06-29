import numpy as np


class ESO:
    def __init__(self, A, B, L):
        self.A = A
        self.B = B
        self.L = L

    def compute_dot(self, eso_estimates, q, u):
        e = q - eso_estimates[0]
        z_dot = self.A @ eso_estimates[:, np.newaxis] + self.B * u + self.L * e
        return z_dot
