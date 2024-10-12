import numpy as np
import scipy.linalg

Qx = np.array([1000, 0, 0, 0, 0.01, 0, 0, 0, 0]).reshape((3, 3))
Qy = np.array([1000, 0, 0, 0, 0.01, 0, 0, 0, 0]).reshape((3, 3))
Q = scipy.linalg.block_diag(Qx, Qy)
print(Q)