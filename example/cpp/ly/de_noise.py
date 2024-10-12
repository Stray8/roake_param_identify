import numpy as np
import matplotlib.pyplot as plt

np.random.seed(5)
gap = 10
Robot_DoF = 7
# The structure is M, q, dq, x, Cdq, tau, g, time

# q = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/drag_data/position.txt')[0::2,:]
# dq = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/drag_data/velocity.txt')[0::2,:]

q = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/simulation_1/position_ly.txt')
dq = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/simulation_1/velocity_file.txt')

# q filter
N = np.shape(q)[0]
D1 = np.hstack((-np.eye(N - 1), np.zeros((N - 1, 1))))
D2 = np.hstack((np.zeros((N - 1, 1)), np.eye(N - 1)))
D_first_order = D1 + D2
delta_first_order = 10
D3 = np.hstack((np.eye(N - 2), np.zeros((N - 2, 2))))
D4 = np.hstack((np.zeros((N - 2, 1)), -2 * np.eye(N - 2), np.zeros((N-2, 1))))
D5 = np.hstack((np.zeros((N - 2, 2)), np.eye(N - 2)))
D_second_order = D3 + D4 + D5
delta_second_order = 100
print("************************")
hat_q = np.linalg.inv(np.eye(N) + delta_first_order * D_first_order.T.dot(D_first_order) + delta_second_order *
                      D_second_order.T.dot(D_second_order)).dot(q)
print("************************")
hat_dq = (hat_q[1:, :] - hat_q[0:-1, :]) / (gap * 1e-3)
print("************************")

ddq = D_second_order.dot(q) / (gap * 1e-3) / (gap * 1e-3)
hat_ddq = (hat_dq[1:, :] - hat_dq[0:-1, :]) / (gap * 1e-3)

np.savetxt('/home/robot/robot/roake_param_identify/build/drag/simulation_1/hat_p.txt', hat_q)
np.savetxt('/home/robot/robot/roake_param_identify/build/drag/simulation_1/hat_dp.txt', hat_dq)
np.savetxt('/home/robot/robot/roake_param_identify/build/drag/simulation_1/hat_ddp.txt', hat_ddq)

plot_dim = 0
# 位置
plt.subplot(421)
plt.plot(np.arange(N), q[:, plot_dim], c='red', label='$q_1$')
plt.plot(np.arange(N), hat_q[:, plot_dim], c='blue', label='$\hat q_1$')
plt.legend()
plt.subplot(422)
plt.plot(np.arange(N), q[:, plot_dim+1], c='red', label='$q_2$')
plt.plot(np.arange(N), hat_q[:, plot_dim+1], c='blue', label='$\hat q_2$')
plt.legend()
plt.subplot(423)
plt.plot(np.arange(N), q[:, plot_dim+2], c='red', label='$q_3$')
plt.plot(np.arange(N), hat_q[:, plot_dim+2], c='blue', label='$\hat q_3$')
plt.legend()
plt.subplot(424)
plt.plot(np.arange(N), q[:, plot_dim+3], c='red', label='$q_4$')
plt.plot(np.arange(N), hat_q[:, plot_dim+3], c='blue', label='$\hat q_4$')
plt.legend()
plt.subplot(425)
plt.plot(np.arange(N), q[:, plot_dim+4], c='red', label='$q_5$')
plt.plot(np.arange(N), hat_q[:, plot_dim+4], c='blue', label='$\hat q_5$')
plt.legend()
plt.subplot(426)
plt.plot(np.arange(N), q[:, plot_dim+5], c='red', label='$q_6$')
plt.plot(np.arange(N), hat_q[:, plot_dim+5], c='blue', label='$\hat q_6$')
plt.legend()
plt.subplot(427)
plt.plot(np.arange(N), q[:, plot_dim+6], c='red', label='$q_7$')
plt.plot(np.arange(N), hat_q[:, plot_dim+6], c='blue', label='$\hat q_7$')
plt.legend()
plt.show()
# 速度
plt.subplot(421)
plt.plot(np.arange(N), dq[:, plot_dim], c='red', label='$dq_1$')
plt.plot(np.arange(N-1), hat_dq[:, plot_dim], c='blue', label='$\hat dq_1$')
plt.legend()
plt.subplot(422)
plt.plot(np.arange(N), dq[:, plot_dim+1], c='red', label='$dq_2$')
plt.plot(np.arange(N-1), hat_dq[:, plot_dim+1], c='blue', label='$\hat dq_2$')
plt.legend()
plt.subplot(423)
plt.plot(np.arange(N), dq[:, plot_dim+2], c='red', label='$dq_3$')
plt.plot(np.arange(N-1), hat_dq[:, plot_dim+2], c='blue', label='$\hat dq_3$')
plt.legend()
plt.subplot(424)
plt.plot(np.arange(N), dq[:, plot_dim+3], c='red', label='$dq_4$')
plt.plot(np.arange(N-1), hat_dq[:, plot_dim+3], c='blue', label='$\hat dq_4$')
plt.legend()
plt.subplot(425)
plt.plot(np.arange(N), dq[:, plot_dim+4], c='red', label='$dq_5$')
plt.plot(np.arange(N-1), hat_dq[:, plot_dim+4], c='blue', label='$\hat dq_5$')
plt.legend()
plt.subplot(426)
plt.plot(np.arange(N), dq[:, plot_dim+5], c='red', label='$dq_6$')
plt.plot(np.arange(N-1), hat_dq[:, plot_dim+5], c='blue', label='$\hat dq_6$')
plt.legend()
plt.subplot(427)
plt.plot(np.arange(N), dq[:, plot_dim+6], c='red', label='$dq_7$')
plt.plot(np.arange(N-1), hat_dq[:, plot_dim+6], c='blue', label='$\hat dq_7$')
plt.legend()
plt.show()
# 加速度
plt.subplot(421)
plt.plot(np.arange(N-2), ddq[:, plot_dim], c='red', label='$ddq_1$')
plt.plot(np.arange(N-2), hat_ddq[:, plot_dim], c='blue', label='$\hat ddq_1$')
plt.legend()
plt.subplot(422)
plt.plot(np.arange(N-2), ddq[:, plot_dim+1], c='red', label='$ddq_2$')
plt.plot(np.arange(N-2), hat_ddq[:, plot_dim+1], c='blue', label='$\hat ddq_2$')
plt.legend()
plt.subplot(423)
plt.plot(np.arange(N-2), ddq[:, plot_dim+2], c='red', label='$ddq_3$')
plt.plot(np.arange(N-2), hat_ddq[:, plot_dim+2], c='blue', label='$\hat ddq_3$')
plt.legend()
plt.subplot(424)
plt.plot(np.arange(N-2), ddq[:, plot_dim+3], c='red', label='$ddq_4$')
plt.plot(np.arange(N-2), hat_ddq[:, plot_dim+3], c='blue', label='$\hat ddq_4$')
plt.legend()
plt.subplot(425)
plt.plot(np.arange(N-2), ddq[:, plot_dim+4], c='red', label='$ddq_5$')
plt.plot(np.arange(N-2), hat_ddq[:, plot_dim+4], c='blue', label='$\hat ddq_5$')
plt.legend()
plt.subplot(426)
plt.plot(np.arange(N-2), ddq[:, plot_dim+5], c='red', label='$ddq_6$')
plt.plot(np.arange(N-2), hat_ddq[:, plot_dim+5], c='blue', label='$\hat ddq_6$')
plt.legend()
plt.subplot(427)
plt.plot(np.arange(N-2), ddq[:, plot_dim+6], c='red', label='$ddq_7$')
plt.plot(np.arange(N-2), hat_ddq[:, plot_dim+6], c='blue', label='$\hat ddq_7$')
plt.legend()
plt.show()