import numpy as np


class ManiuplatorModel:
    def __init__(self, Tp):
        self.Tp = Tp
        self.l1 = 0.5
        self.r1 = self.l1/2.
        self.m1 = 1.
        self.l2 = 0.5
        self.r2 = self.l2/2.
        self.m2 = 1.
        self.I_1 = 1 / 12 * self.m1 * (3 * self.r1 ** 2 + self.l1 ** 2)
        self.I_2 = 1 / 12 * self.m2 * (3 * self.r2 ** 2 + self.l2 ** 2)
        self.m3 = 1.0
        self.r3 = 0.3
        self.I_3 = 2. / 5 * self.m3 * self.r3 ** 2

        self.alfa = self.m1 * self.r1 ** 2 + self.I_1 + self.m2 * (self.l1 ** 2 + self.r2 ** 2) + self.I_2 + self.m3 * (self.l1 ** 2 + self.l2 ** 2) + self.I_3
        self.beta = self.m2 * self.l1 * self.r2 + self.m3 * self.l1 * self.l2
        self.gamma = self.m2 * self.r2 ** 2 + self.I_2 + self.m3 * self.l2 ** 2 + self.I_3


    def M(self, x):
        """
        Please implement the calculation of the mass matrix, according to the model derived in the exercise
        (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x

        m11 = self.alfa + 2 * self.beta * np.cos(q2)
        m12 = self.gamma + self.beta * np.cos(q2)
        m21 = self.gamma + self.beta * np.cos(q2)
        m22 = self.gamma

        M = np.array([[m11, m12], [m21, m22]])

        return M

    def C(self, x):
        """
        Please implement the calculation of the Coriolis and centrifugal forces matrix, according to the model derived
        in the exercise (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x
        c11 = - self.beta * np.sin(q2) * q2_dot
        c12 = - self.beta * np.sin(q2) * (q1_dot + q2_dot)
        c21 = self.beta * np.sin(q2) * q1_dot
        c22 = 0
        C = np.array([[c11, c12], [c21, c22]])
        return C

    def x_dot(self):
        # TODO zeby mma controller mogl uzywac choose model zwracaj xdot
        pass
