import numpy as np
from env.PolishWithFraction import Polish
from algorithm.OneStep_Impedance_Cons import constraintMpc
import time
from algorithm.QP2PVC import Qp2pvc

#  generate the desired trajectory
#  螺旋轨迹
Period = 0.02
B_steps = int(2 / Period)
Steps = 4 * B_steps
D_trajectory = np.zeros((Steps + 1, 4))
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
        D_trajectory[i, 1] = -0
        D_trajectory[i, 2] = D_trajectory[i - 1, 2]
        D_trajectory[i, 3] = 0
    elif i < B_steps * 4:
        D_trajectory[i, 0] = D_trajectory[i - 1, 0]
        D_trajectory[i, 1] = 0
        D_trajectory[i, 2] = D_trajectory[B_steps * 3 - 1, 2] - v * (i - B_steps * 3) * Period
        D_trajectory[i, 3] = -0

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

D_trajectory_complementary = []
complementary_num = 10
for i in range(complementary_num):
    D_trajectory_complementary.append(D_trajectory[-1, :])
D_trajectory_complementary = np.array(D_trajectory_complementary)
D_trajectory = np.vstack((D_trajectory, D_trajectory_complementary))
#  controller hyper-parameters
A = np.array([0, 1, 0, 0, 0, 0, 0, 0, 0]).reshape((3, 3))
b = np.array([0, 1, 0]).reshape((3, 1))
Q = np.array([50, 0, 0, 0, 0, 0, 0, 0, 0]).reshape((3, 3))
R = np.array([0.0000001, 0, 0, 0, 0.0000001, 0, 0, 0, 0.0000001]).reshape((3, 3))
impedance_cons = np.array([150.0, 200.0, 1.0])

algorithm_x = constraintMpc(A=A, b=b, Q=Q, R=R, Period=Period, impedance_cons=impedance_cons)
algorithm_y = constraintMpc(A=A, b=b, Q=Q, R=R, Period=Period, impedance_cons=impedance_cons)
Task_env = Polish(Period=Period, mu_s=0., mu_c=0., c=0, D_trajectory=D_trajectory)
px, vx, py, vy = D_trajectory[0]
f = np.zeros(2)

H = 5
cons_x = np.array([0.3, 0.46, -0.1, 0.1])
cons_y = np.array([-0.1, 0.08, -0.1, 0.1])
W = np.eye(H)
algorithm2_x = Qp2pvc(H=H, cons=cons_x, W=W)
algorithm2_y = Qp2pvc(H=H, cons=cons_y, W=W)

for i in range(Steps):
    s_x = np.array([px, vx, f[0]])
    s_y = np.array([py, vy, f[1]])
    t1 = time.time()

    s_r_x = np.array([D_trajectory[i + 1, 0], D_trajectory[i + 1, 1], 0])
    s_r_y = np.array([D_trajectory[i + 1, 2], D_trajectory[i + 1, 3], 0])

    u_x = algorithm_x.controller(s_x, s_r_x)
    u_y = algorithm_y.controller(s_y, s_r_y)

    print('i is ', i)
    print('pvx and dpvx are ', s_x[0:2], D_trajectory[i + 1, 0:2])
    print('pvy and dpvy are ', s_y[0:2], D_trajectory[i + 1, 2:4])

    beta = np.array([Period / 2, 0, 0]).reshape(3, 1)
    _Ax = A * Period + np.eye(3) + Period * (b + beta).dot(u_x.T)
    _Bx = u_x[2, 0] * Period * (b + beta)
    _Cx = Period * (b + beta).dot(u_x.T)
    _srx_matrix = np.hstack((D_trajectory[i:i+H, 0:2], np.zeros(H).reshape(-1, 1)))
    f_x = algorithm2_x.computing_f_seq(_Ax, _Bx, _Cx, s_x, _srx_matrix)[0, 0]

    _Ay = A * Period + np.eye(3) + Period * (b + beta).dot(u_y.T)
    _By = u_y[2, 0] * Period * (b + beta)
    _Cy = Period * (b + beta).dot(u_y.T)
    _sry_matrix = np.hstack((D_trajectory[i:i + H, 2:4], np.zeros(H).reshape(-1, 1)))
    f_y = algorithm2_y.computing_f_seq(_Ay, _By, _Cy, s_y, _sry_matrix)[0, 0]

    t2 = time.time()
    print('using time for QPs: ', t2 - t1)
    print('fx and fy are ', f_x, f_y)
    u = np.vstack((u_x, u_y)).reshape(-1)

    px, vx, py, vy, fx, fy = Task_env.step(np.array([px, vx, py, vy]), u, D_trajectory[i + 1], np.array([f_x, f_y]))
    Task_env.plot_trajectory()

Task_env.plot_trajectory(Stop_Flag=True)
