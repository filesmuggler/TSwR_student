import numpy as np


class ManiuplatorModel:
    def __init__(self, Tp, m3, r3):
        self.Tp = Tp
        self.l1 = 0.5
        self.r1 = 0.01
        self.m1 = 1.
        self.l2 = 0.5
        self.r2 = 0.01
        self.m2 = 1.
        self.I_1 = 1 / 12 * self.m1 * (3 * self.r1 ** 2 + self.l1 ** 2)
        self.I_2 = 1 / 12 * self.m2 * (3 * self.r2 ** 2 + self.l2 ** 2)
        self.m3 = m3  # planar 0.2
        self.r3 = r3  # planar 0.1
        self.I_3 = 2. / 5 * self.m3 * self.r3 ** 2

        # centers of masses:
        self.com1 = self.l1 / 2
        self.com2 = self.l2 / 2

        self.alfa = self.m1 * self.com1 ** 2 + self.I_1 + self.m2 * (self.l1 ** 2 + self.com2 ** 2) + self.I_2 + self.m3 * (self.l1 ** 2 + self.l2 ** 2) + self.I_3
        self.beta = self.m2 * self.l1 * self.com2 + self.m3 * self.l1 * self.l2
        self.gamma = self.m2 * self.com2 ** 2 + self.I_2 + self.m3 * self.l2 ** 2 + self.I_3


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

    def x_dot(self, x, u):
        invM = np.linalg.inv(self.M(x))
        zeros = np.zeros((2, 2), dtype=np.float32)
        A = np.concatenate([np.concatenate([zeros, np.eye(2)], 1), np.concatenate([zeros, -invM @ self.C(x)], 1)], 0)
        b = np.concatenate([zeros, invM], 0)
        return A @ x[:, np.newaxis] + b @ u
