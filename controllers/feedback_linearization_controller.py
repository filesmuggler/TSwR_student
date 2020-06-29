import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp, 4.0, 1.0)
        self.k_p = [[-3, 0.],
                    [0., -0.7]]
        self.k_d = [[-0.7, 0.],
                    [0., -2]]

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """
        q_dot = x[2:, np.newaxis]
        q = x[0:1, np.newaxis]

        v = q_d_ddot + self.k_d * (q_dot - q_d_dot) + self.k_p * (q - q_d)
        return self.model.M(x) @ v + self.model.C(x) @ q_dot
