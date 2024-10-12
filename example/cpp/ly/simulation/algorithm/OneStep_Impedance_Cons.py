import numpy as np
import cvxopt
from cvxopt import matrix, solvers

class constraintMpc:
    def __init__(self, A=np.array([0, 1, 0, 0, 0, 0, 0, 0, 0]).reshape((3, 3)),
                 b=np.array([0, 1, 0]).reshape((3, 1)),
                 Q=np.array([50, 0, 0, 0, 10, 0, 0, 0, 0]).reshape((3, 3)),
                 R=np.array([0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]).reshape((3, 3)),
                 Period=0.01, impedance_cons=np.array([10.0, 50.0, 1.0])):
        self.A = A
        self.b = b + np.array([0.5 * Period, 0, 0]).reshape((3, 1))
        self.Q = Q
        self.R = R
        self.Period = Period
        self.impedance_cons = impedance_cons

    def controller(self, s, s_r):
        k_min = self.impedance_cons[0]
        k_max = self.impedance_cons[1]
        m = self.impedance_cons[2]

        I = np.eye(3)
        s = s.reshape(3, 1)
        s_r = s_r.reshape(3, 1)
        tilde_s = s - s_r
        T = self.Period
        Q = matrix(2 * (T * T * np.dot(np.dot(np.dot(tilde_s, self.b.T), self.Q), np.dot(self.b, tilde_s.T)) + self.R))
        p = matrix(2 * np.dot(np.dot(np.dot(T * tilde_s, self.b.T), self.Q), (np.dot(T * self.A + I, s) - s_r)).reshape(-1))
        G = matrix(np.array([[m, 0.0, 0.0], [-m, 0.0, 0.0]]))
        h = matrix([-k_min, k_max])

        A = matrix(np.array([[2 * np.sqrt(m) / np.sqrt(k_min), -1, 0], [0, 0, 1]]))
        b = matrix([0, 1 / m])
        sol = solvers.qp(Q, p, G, h, A, b)
        return np.array(sol['x'])

Period = 0.05
#  controller hyper-parameters
A = np.array([0, 1, 0, 0, 0, 0, 0, 0, 0]).reshape((3, 3))
b = np.array([0, 1, 0]).reshape((3, 1))
Q = np.array([5, 0, 0, 0, 0, 0, 0, 0, 0]).reshape((3, 3))
R = np.array([0.000000001, 0, 0, 0, 0.000000001, 0, 0, 0, 0.000000001]).reshape((3, 3))
impedance_cons = np.array([50.0, 200.0, 1.0])
algorithm = constraintMpc(A=A, b=b, Q=Q, R=R, Period=Period, impedance_cons=impedance_cons)

pr = np.eye(7)
vr = 0.5 * np.eye(7)
s_r_0 = np.array([1, 0.5, 0])
s_r_1 = np.array([1, 0.5, 0])
s_r_2 = np.array([1, 0.5, 0])
s_r_3 = np.array([1, 0.5, 0])
s_r_4 = np.array([1, 0.5, 0])
s_r_5 = np.array([1, 0.5, 0])
s_r_6 = np.array([1, 0.5, 0])

f = np.zeros(7)

def out(x):
    p = x[0,:]
    dp = x[1,:]

    x0 = p[0]
    x1 = p[1]
    x2 = p[2]
    x3 = p[3]
    x4 = p[4]
    x5 = p[5]
    x6 = p[6]

    v0 = dp[0]
    v1 = dp[1]
    v2 = dp[2]
    v3 = dp[3]
    v4 = dp[4]
    v5 = dp[5]
    v6 = dp[6]

    s_0 = np.array([x0, v0, f[0]])
    s_1 = np.array([x1, v1, f[1]])
    s_2 = np.array([x2, v2, f[2]])
    s_3 = np.array([x3, v3, f[3]])
    s_4 = np.array([x4, v4, f[4]])
    s_5 = np.array([x5, v5, f[5]])
    s_6 = np.array([x6, v6, f[6]])

    y0 = algorithm.controller(s_0, s_r_0)
    y1 = algorithm.controller(s_1, s_r_1)
    y2 = algorithm.controller(s_2, s_r_2)
    y3 = algorithm.controller(s_3, s_r_3)
    y4 = algorithm.controller(s_4, s_r_4)
    y5 = algorithm.controller(s_5, s_r_5)
    y6 = algorithm.controller(s_6, s_r_6)

    y = np.array([y0, y1, y2, y3, y4, y5, y6])
# x = np.array([1, 2, 3, 4, 5, 6, 7])
# dx = 0.1 * np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
# input = np.vstack([x, dx])
# print("input is\n", input)
# u = out(input)
# print("u is\n", u)

# print("u is:", u)
