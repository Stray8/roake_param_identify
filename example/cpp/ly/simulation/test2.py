import numpy as np
from cvxopt import matrix, solvers

A = np.array([0, 1, 0, 0, 0, 0, 0, 0, 0]).reshape(3, 3)
I = np.eye(3)
T = 0.01
A_ = A * T + I
print(A_)
print(np.linalg.eig(A_))

