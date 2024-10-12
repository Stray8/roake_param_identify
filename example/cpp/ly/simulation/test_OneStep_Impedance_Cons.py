import numpy as np
from env.PolishWithFraction import Polish
from algorithm.OneStep_Impedance_Cons import constraintMpc
import time

#  generate the desired trajectory
#  螺旋轨迹
Period = 0.05
B_steps = int(2 / Period)
Steps = 4 * B_steps
D_trajectory = np.zeros((Steps + 1, 4))
p_Th_x = 0.48
p_Th_y = 1
v = 0.05
for i in range(Steps):
    if i < B_steps:
        D_trajectory[i, 0] = v * i * Period + 0.4
        D_trajectory[i, 1] = 0
        D_trajectory[i, 2] = 0
        D_trajectory[i, 3] = 0
    elif i < B_steps * 2:
        D_trajectory[i, 0] = D_trajectory[i - 1, 0]
        D_trajectory[i, 1] = 0
        D_trajectory[i, 2] = v * Period * (i - B_steps)
        D_trajectory[i, 3] = 0
    elif i < B_steps * 3:
        D_trajectory[i, 0] = D_trajectory[B_steps * 2 - 1, 0] - v * (i - B_steps * 2) * Period
        D_trajectory[i, 1] = 0
        D_trajectory[i, 2] = D_trajectory[i - 1, 2]
        D_trajectory[i, 3] = 0
    elif i < B_steps * 4:
        D_trajectory[i, 0] = D_trajectory[i - 1, 0]
        D_trajectory[i, 1] = 0
        D_trajectory[i, 2] = D_trajectory[B_steps * 3 - 1, 2] - v * (i - B_steps * 3) * Period
        D_trajectory[i, 3] = 0

D_trajectory[-1, 0] = D_trajectory[-2, 0]
D_trajectory[-1, 1] = D_trajectory[-2, 1]
D_trajectory[-1, 2] = D_trajectory[-2, 2]
D_trajectory[-1, 3] = D_trajectory[-2, 3]


'''
p_Th_x = 0.015
p_Th_y = 0.2

for i in range(Steps):
    D_trajectory[i, 0] = v * i * Period * np.cos(w * i * Period)
    D_trajectory[i, 1] = v * np.cos(w * i * Period) - w * v * i * Period * np.sin(w * i * Period)
    if D_trajectory[i, 0] > p_Th_x:
        D_trajectory[i, 0] = p_Th_x
        D_trajectory[i, 1] = 0
    D_trajectory[i, 2] = v * i * Period * np.sin(w * i * Period)
    D_trajectory[i, 3] = v * np.sin(w * i * Period) + w * v * i * Period * np.cos(w * i * Period)
    if D_trajectory[i, 2] > p_Th_y:
        D_trajectory[i, 2] = p_Th_y
        D_trajectory[i, 3] = 0
D_trajectory[-1, 1] = 0
D_trajectory[-1, 3] = 0
'''

#  controller hyper-parameters
A = np.array([0, 1, 0, 0, 0, 0, 0, 0, 0]).reshape((3, 3))
b = np.array([0, 1, 0]).reshape((3, 1))
Q = np.array([5, 0, 0, 0, 0, 0, 0, 0, 0]).reshape((3, 3))
R = np.array([0.000000001, 0, 0, 0, 0.000000001, 0, 0, 0, 0.000000001]).reshape((3, 3))
impedance_cons = np.array([50.0, 200.0, 1.0])

algorithm_x = constraintMpc(A=A, b=b, Q=Q, R=R, Period=Period, impedance_cons=impedance_cons)
algorithm_y = constraintMpc(A=A, b=b, Q=Q, R=R, Period=Period, impedance_cons=impedance_cons)
Task_env = Polish(Period=Period, mu_s=0., mu_c=0., c=0, D_trajectory=D_trajectory)
px, vx, py, vy = D_trajectory[0]
f = np.zeros(2)

for i in range(Steps):
    s_x = np.array([px, vx, f[0]])
    s_y = np.array([py, vy, f[1]])
    t1 = time.time()

    s_r_x = np.array([D_trajectory[i + 1, 0], D_trajectory[i + 1, 1], 0])
    s_r_y = np.array([D_trajectory[i + 1, 2], D_trajectory[i + 1, 3], 0])

    u_x = algorithm_x.controller(s_x, s_r_x)
    u_y = algorithm_y.controller(s_y, s_r_y)
    t2 = time.time()
    print('using time: ', t2 - t1)
    print('i is ', i)
    print('pvx and dpvx are ', s_x[0:2], D_trajectory[i + 1, 0:2])
    print('pvy and dpvy are ', s_y[0:2], D_trajectory[i + 1, 2:4])
    u = np.vstack((u_x, u_y)).reshape(-1)

    px, vx, py, vy, fx, fy = Task_env.step(np.array([px, vx, py, vy]), u, D_trajectory[i + 1], np.array([0, 0]))
    # Task_env.plot_trajectory()

Task_env.plot_trajectory(Stop_Flag=True)
