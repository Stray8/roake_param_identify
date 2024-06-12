import numpy as np
import matplotlib.pyplot as plt

np.random.seed(5)
gap = 10
Robot_DoF = 7
# The structure is M, q, dq, x, Cdq, tau, g, time

# inertia = np.loadtxt('/home/robot/robot/roake_param_identify/build/collect/inertia.txt')[0::gap, :]
# coriolis = np.loadtxt('/home/robot/robot/roake_param_identify/build/collect/coriolis.txt')[0::gap, :]
# gravity = np.loadtxt('/home/robot/robot/roake_param_identify/build/collect/gravity.txt')[0::gap, :]
# torque = np.loadtxt('/home/robot/robot/roake_param_identify/build/collect/torque.txt')[0::gap, :]
# q = np.loadtxt('/home/robot/robot/roake_param_identify/build/collect/position.txt')[0::gap, :]
# dq = np.loadtxt('/home/robot/robot/roake_param_identify/build/collect/velocity.txt')[0::gap, :]

inertia = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_1/inertia_file.txt')[0::gap, :]
coriolis = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_1/coriolis_file.txt')[0::gap, :]
gravity = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_1/gravity_file.txt')[0::gap, :]
torque = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_1/torque_file.txt')[0::gap, :]
q = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_1/position_ly.txt')[0::gap, :]
dq = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_1/velocity_file.txt')[0::gap, :]

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

hat_q = np.linalg.inv(np.eye(N) + delta_first_order * D_first_order.T.dot(D_first_order) + delta_second_order *
                      D_second_order.T.dot(D_second_order)).dot(q)
hat_dq = (hat_q[1:, :] - hat_q[0:-1, :]) / (gap * 1e-3)

ddq = D_second_order.dot(q) / (gap * 1e-3) / (gap * 1e-3)
hat_ddq = (hat_dq[1:, :] - hat_dq[0:-1, :]) / (gap * 1e-3)
hat_i = np.linalg.inv(np.eye(N) + delta_first_order * D_first_order.T.dot(D_first_order) + delta_second_order *
                      D_second_order.T.dot(D_second_order)).dot(inertia)
hat_c = np.linalg.inv(np.eye(N) + delta_first_order * D_first_order.T.dot(D_first_order) + delta_second_order *
                      D_second_order.T.dot(D_second_order)).dot(coriolis)
hat_g = np.linalg.inv(np.eye(N) + delta_first_order * D_first_order.T.dot(D_first_order) + delta_second_order *
                      D_second_order.T.dot(D_second_order)).dot(gravity)
hat_tau = np.linalg.inv(np.eye(N) + delta_first_order * D_first_order.T.dot(D_first_order) + delta_second_order *
                        D_second_order.T.dot(D_second_order)).dot(torque)

# Construct training set
X = np.hstack((hat_q[0:-2, :], hat_dq[0:-1, :], hat_ddq[:, :])) # 要保证hat_q hat_dq hat_ddq的行数相同
Y = np.zeros((N-2, Robot_DoF)) # 定义一个N-2行Robot_Dof列的零矩阵
model_tau = np.zeros((N - 2, Robot_DoF)) # 定义一个N-2行Robot_Dof列的零矩阵
# M_C = np.zeros((N - 2, Robot_DoF)) # 定义一个N-2行Robot_Dof列的零矩阵
for i in range(N - 2): # i是从0开始的
    hat_tau_i = hat_tau[i, :]
    hat_c_i = hat_c[i, :]
    hat_i_i = hat_i[i, :]
    hat_g_i = hat_g[i, :]
    # Y[i, :] = hat_tau_i - hat_c_i - hat_i_i
    Y[i, :] = hat_tau_i - hat_g_i - hat_c_i - hat_i_i

training_set = np.hstack((X, Y))
# np.savetxt('/home/robot/robot/roake_param_identify/build/collect/trainingSet.txt', training_set)
np.savetxt('/home/robot/robot/roake_param_identify/build/simulation_data_1/trainingSet_ly.txt', training_set)

static_error = q[-1, :] - q[0, :]
print('static error is ', static_error)
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
# inertia力矩
plt.subplot(421)
plt.plot(np.arange(N), inertia[:, plot_dim], c='red', label='$inertia_1$')
plt.plot(np.arange(N), hat_i[:, plot_dim], c='blue', label='$\hat i_1$')
plt.legend()
plt.subplot(422)
plt.plot(np.arange(N), inertia[:, plot_dim+1], c='red', label='$inertia_2$')
plt.plot(np.arange(N), hat_i[:, plot_dim+1], c='blue', label='$\hat i_2$')
plt.legend()
plt.subplot(423)
plt.plot(np.arange(N), inertia[:, plot_dim+2], c='red', label='$inertia_3$')
plt.plot(np.arange(N), hat_i[:, plot_dim+2], c='blue', label='$\hat i_3$')
plt.legend()
plt.subplot(424)
plt.plot(np.arange(N), inertia[:, plot_dim+3], c='red', label='$inertia_4$')
plt.plot(np.arange(N), hat_i[:, plot_dim+3], c='blue', label='$\hat i_4$')
plt.legend()
plt.subplot(425)
plt.plot(np.arange(N), inertia[:, plot_dim+4], c='red', label='$inertia_5$')
plt.plot(np.arange(N), hat_i[:, plot_dim+4], c='blue', label='$\hat i_5$')
plt.legend()
plt.subplot(426)
plt.plot(np.arange(N), inertia[:, plot_dim+5], c='red', label='$inertia_6$')
plt.plot(np.arange(N), hat_i[:, plot_dim+5], c='blue', label='$\hat i_6$')
plt.legend()
plt.subplot(427)
plt.plot(np.arange(N), inertia[:, plot_dim+6], c='red', label='$inertia_7$')
plt.plot(np.arange(N), hat_i[:, plot_dim+6], c='blue', label='$\hat i_7$')
plt.legend()
plt.show()
# coriolis力矩
plt.subplot(421)
plt.plot(np.arange(N), coriolis[:, plot_dim], c='red', label='$coriolis_1$')
plt.plot(np.arange(N), hat_c[:, plot_dim], c='blue', label='$\hat c_1$')
plt.legend()
plt.subplot(422)
plt.plot(np.arange(N), coriolis[:, plot_dim+1], c='red', label='$coriolis_2$')
plt.plot(np.arange(N), hat_c[:, plot_dim+1], c='blue', label='$\hat c_2$')
plt.legend()
plt.subplot(423)
plt.plot(np.arange(N), coriolis[:, plot_dim+2], c='red', label='$coriolis_3$')
plt.plot(np.arange(N), hat_c[:, plot_dim+2], c='blue', label='$\hat c_3$')
plt.legend()
plt.subplot(424)
plt.plot(np.arange(N), coriolis[:, plot_dim+3], c='red', label='$coriolis_4$')
plt.plot(np.arange(N), hat_c[:, plot_dim+3], c='blue', label='$\hat c_4$')
plt.legend()
plt.subplot(425)
plt.plot(np.arange(N), coriolis[:, plot_dim+4], c='red', label='$coriolis_5$')
plt.plot(np.arange(N), hat_c[:, plot_dim+4], c='blue', label='$\hat c_5$')
plt.legend()
plt.subplot(426)
plt.plot(np.arange(N), coriolis[:, plot_dim+5], c='red', label='$coriolis_6$')
plt.plot(np.arange(N), hat_c[:, plot_dim+5], c='blue', label='$\hat c_6$')
plt.legend()
plt.subplot(427)
plt.plot(np.arange(N), coriolis[:, plot_dim+6], c='red', label='$coriolis_7$')
plt.plot(np.arange(N), hat_c[:, plot_dim+6], c='blue', label='$\hat c_7$')
plt.legend()
plt.show()
# gravity力矩
plt.subplot(421)
plt.plot(np.arange(N), gravity[:, plot_dim], c='red', label='$gravity_1$')
plt.plot(np.arange(N), hat_g[:, plot_dim], c='blue', label='$\hat g_1$')
plt.legend()
plt.subplot(422)
plt.plot(np.arange(N), gravity[:, plot_dim+1], c='red', label='$gravity_2$')
plt.plot(np.arange(N), hat_g[:, plot_dim+1], c='blue', label='$\hat g_2$')
plt.legend()
plt.subplot(423)
plt.plot(np.arange(N), gravity[:, plot_dim+2], c='red', label='$gravity_3$')
plt.plot(np.arange(N), hat_g[:, plot_dim+2], c='blue', label='$\hat g_3$')
plt.legend()
plt.subplot(424)
plt.plot(np.arange(N), gravity[:, plot_dim+3], c='red', label='$gravity_4$')
plt.plot(np.arange(N), hat_g[:, plot_dim+3], c='blue', label='$\hat g_4$')
plt.legend()
plt.subplot(425)
plt.plot(np.arange(N), gravity[:, plot_dim+4], c='red', label='$gravity_5$')
plt.plot(np.arange(N), hat_g[:, plot_dim+4], c='blue', label='$\hat g_5$')
plt.legend()
plt.subplot(426)
plt.plot(np.arange(N), gravity[:, plot_dim+5], c='red', label='$gravity_6$')
plt.plot(np.arange(N), hat_g[:, plot_dim+5], c='blue', label='$\hat g_6$')
plt.legend()
plt.subplot(427)
plt.plot(np.arange(N), gravity[:, plot_dim+6], c='red', label='$gravity_7$')
plt.plot(np.arange(N), hat_g[:, plot_dim+6], c='blue', label='$\hat g_7$')
plt.legend()
plt.show()
# torque力矩
plt.subplot(421)
plt.plot(np.arange(N), torque[:, plot_dim], c='red', label='$torque_1$')
plt.plot(np.arange(N), hat_tau[:, plot_dim], c='blue', label='$\hat t_1$')
plt.legend()
plt.subplot(422)
plt.plot(np.arange(N), torque[:, plot_dim+1], c='red', label='$torque_2$')
plt.plot(np.arange(N), hat_tau[:, plot_dim+1], c='blue', label='$\hat t_2$')
plt.legend()
plt.subplot(423)
plt.plot(np.arange(N), torque[:, plot_dim+2], c='red', label='$torque_3$')
plt.plot(np.arange(N), hat_tau[:, plot_dim+2], c='blue', label='$\hat t_3$')
plt.legend()
plt.subplot(424)
plt.plot(np.arange(N), torque[:, plot_dim+3], c='red', label='$torque_4$')
plt.plot(np.arange(N), hat_tau[:, plot_dim+3], c='blue', label='$\hat t_4$')
plt.legend()
plt.subplot(425)
plt.plot(np.arange(N), torque[:, plot_dim+4], c='red', label='$torque_5$')
plt.plot(np.arange(N), hat_tau[:, plot_dim+4], c='blue', label='$\hat t_5$')
plt.legend()
plt.subplot(426)
plt.plot(np.arange(N), torque[:, plot_dim+5], c='red', label='$torque_6$')
plt.plot(np.arange(N), hat_tau[:, plot_dim+5], c='blue', label='$\hat t_6$')
plt.legend()
plt.subplot(427)
plt.plot(np.arange(N), torque[:, plot_dim+6], c='red', label='$torque_7$')
plt.plot(np.arange(N), hat_tau[:, plot_dim+6], c='blue', label='$\hat t_7$')
plt.legend()
plt.show()