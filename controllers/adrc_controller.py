import numpy as np
from .controller import Controller


class ADRController(Controller):
    def __init__(self, b, kp, kd):
        self.b = b
        self.kp = kp
        self.kd = kd

    def calculate_control(self, q, q_d, q_d_dot, q_d_ddot, eso_estimates):
        q_estimate = eso_estimates[0]
        q_dot_estimate = eso_estimates[1]
        f_estimate = eso_estimates[2]
        e = q_d - q_estimate
        e_dot = q_d_dot - q_dot_estimate
        v = self.kp * e + self.kd * e_dot + q_d_ddot
        u = (v - f_estimate) / self.b
        return u
