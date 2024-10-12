import numpy as np
from env.PolishWithFraction import Polish


#  generate the desired trajectory
Period = 0.01
v = 0.5
w = 360 / 180 * np.pi
Steps = 200
D_trajectory = np.zeros((Steps, 4))

for i in range(Steps):
    D_trajectory[i, 0] = v * i * Period * np.cos(w * i * Period)
    D_trajectory[i, 1] = v * np.cos(w * i * Period) - w * v * i * Period * np.sin(w * i * Period)
    D_trajectory[i, 2] = v * i * Period * np.sin(w * i * Period)
    D_trajectory[i, 3] = v * np.sin(w * i * Period) + w * v * i * Period * np.cos(w * i * Period)

D_trajectory[-1, 1] = 0
D_trajectory[-1, 3] = 0

Task_env = Polish(Period=Period, D_Trajectory=D_trajectory, mu_s=0.5, mu_c=0.4, c=0.2)
k = 500
m = 1
d = 2 * np.sqrt(k)
u = np.array([-k/m, -d/m, 1/m, -k/m, -d/m, 1/m])
position = Task_env.position

for i in range(Steps + 100):
    position, _ = Task_env.step(position, u)
    Task_env.plot_trajectory()
