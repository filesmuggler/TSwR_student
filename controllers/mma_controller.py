import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # Use parameters from manipulators/mm_planar_2dof.py
        self.models = [ManiuplatorModel(Tp, 0.1, 0.05),
                       ManiuplatorModel(Tp, 0.01, 0.01),
                       ManiuplatorModel(Tp, 1., 0.3)]
        self.k_p = [[-1.3, 0.],
                    [0., 0.7]]
        self.k_d = [[0.7, 0.],
                    [0., 2]]
        self.i = 0

    def choose_model(self, x, u, x_dot):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        error = [1e6, 1e6, 1e6]
        for idx, model in enumerate(self.models):
            print(idx)
            x_dot_model = model.x_dot(x, u)
            error[idx] = np.sum(abs(x_dot - x_dot_model))
        print(error)
        self.i = np.argmin(error)

        print("\nchosen model: ", self.i, "\n")


    # TODO: ask about situations when it does not adapt quick enough? sometimes perfect, sometimes not with poly trajectory!!!

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):

        q_dot = x[2:, np.newaxis]
        q = x[0:1, np.newaxis]

        v = q_d_ddot + self.k_d * (q_dot - q_d_dot) + self.k_p * (q - q_d)

        M = self.models[self.i].M(x)

        return M @ (v + np.linalg.inv(M) @ self.models[self.i].C(x) @ q_dot)
