import numpy as np
import matplotlib.pyplot as plt
import math

t = np.arange(0, 8, 0.01)
gap = 10
delta_angle0 = np.pi / 50.0 * (1 - np.cos(np.pi / 4 * 1 * t))
delta_angle1 = np.pi / 50.0 * (1 - np.cos(np.pi / 4 * 3 * t)) + np.pi / 6
delta_angle2 = np.pi / 50.0 * (1 - np.cos(np.pi / 2 * 1 * t))
delta_angle3 = np.pi / 70.0 * (1 - np.cos(np.pi * t)) + np.pi / 3
delta_angle4 = np.pi / 60.0 * (1 - np.cos(np.pi * t))
delta_angle5 = np.pi / 60.0 * (1 - np.cos(np.pi / 2 * 4 * t)) + np.pi / 2
delta_angle6 = np.pi / 70.0 * (1 - np.cos(np.pi / 2 * 3 * t))
error_std = t - t
index_8 = int(8/0.01)
delta_angle0[index_8:] = 0
delta_angle1[index_8:] = np.pi / 6
delta_angle2[index_8:] = 0
delta_angle3[index_8:] = np.pi / 3
delta_angle4[index_8:] = 0
delta_angle5[index_8:] = np.pi / 2
delta_angle6[index_8:] = 0

# dp = np.hstack((delta_angle0, delta_angle1, delta_angle2, delta_angle3, delta_angle4, delta_angle5, delta_angle6))
# p_pd = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/joint/s4/position_error_ly.txt')[0::gap,:]
# ep_pd = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_joint/PD/position_error_ly.txt')[0::gap,:]

ep_pd = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/joint/PD/position_error_ly.txt')[0::gap,:]

# 第一次训练结果
# p = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_1/position_ly.txt')[0::gap,:]
# ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_1/position_error_ly.txt')[0::gap,:]
# 第二次训练结果
# p = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_2/position_ly.txt')[0::gap,:]
# ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_2/position_error_ly.txt')[0::gap,:]
# 第三次训练结果
# p = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_3/position_ly.txt')[0::gap,:]
# ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_3/position_error_ly.txt')[0::gap,:]
# 第四次训练结果
# p = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_4/position_ly.txt')[0::gap,:]
# ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_4/position_error_ly.txt')[0::gap,:]

# simulation_1
# p = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_1/position_ly.txt')[0::gap,:]
# ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_1/position_error_ly.txt')[0::gap,:]
# simulation_2
# p = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_2/position_ly.txt')[0::gap,:]
# ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_2/position_error_ly.txt')[0::gap,:]
# simulation_3
# p = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_3/position_ly.txt')[0::gap,:]
# ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_3/position_error_ly.txt')[0::gap,:]
# simulation_4
# p = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_4/position_ly.txt')[0::gap,:]
# ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_4/position_error_ly.txt')[0::gap,:]
# simulation_5
# p = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_5/position_ly.txt')[0::gap,:]
# ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_5/position_error_ly.txt')[0::gap,:]
# simulation_5
# p = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_joint/s4/position_ly.txt')[0::gap,:]
# ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_joint/s4/position_error_ly.txt')[0::gap,:]

ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/joint/s4/position_error_ly.txt')[0::gap,:]

# plt.subplot(4,2,1)
# plt.plot(p[:,0], c='red',label='$p_1$')
# plt.plot(p_pd[:,0], c='purple',label='$p_{pd1}$')
# plt.plot(delta_angle0, c='green',label='$dp_1$')
# plt.ylabel('angle/rad')
# # plt.xlabel('steps')
# plt.legend()
# plt.subplot(4,2,2)
# plt.plot(p[:,1], c='red',label='$p_2$')
# plt.plot(p_pd[:,1], c='purple',label='$p_{pd2}$')
# plt.plot(delta_angle1, c='green',label='$dp_2$')
# plt.ylabel('angle/rad')
# plt.xlabel('steps')
# plt.legend()
# plt.subplot(4,2,3)
# plt.plot(p[:,2], c='red',label='$p_3$')
# plt.plot(p_pd[:,2], c='purple',label='$p_{pd3}$')
# plt.plot(delta_angle2, c='green',label='$dp_3$')
# # plt.ylabel('angle/rad')
# plt.xlabel('steps')
# plt.legend()
# plt.subplot(4,2,4)
# plt.plot(p[:,3], c='red',label='$p_4$')
# plt.plot(p_pd[:,3], c='purple',label='$p_{pd4}$')
# plt.plot(delta_angle3, c='green',label='$dp_4$')
# plt.ylabel('angle/rad')
# plt.xlabel('steps')
# plt.legend()
# plt.subplot(4,2,5)
# plt.plot(p[:,4], c='red',label='$p_5$')
# plt.plot(p_pd[:,4], c='purple',label='$p_pd5$')
# plt.plot(delta_angle4, c='green',label='$dp_5$')
# # plt.ylabel('angle/rad')
# plt.xlabel('steps')
# plt.legend()
# plt.subplot(4,2,6)
# plt.plot(p[:,5], c='red',label='$p_6$')
# plt.plot(p_pd[:,5], c='purple',label='$p_{pd6}$')
# plt.plot(delta_angle5, c='green',label='$dp_6$')
# plt.ylabel('angle/rad')
# plt.xlabel('steps')
# plt.legend()
# plt.subplot(4,2,7)
# plt.plot(p[:,6], c='red',label='$p_7$')
# plt.plot(p_pd[:,6], c='purple',label='$p_{pd7}$')
# plt.plot(delta_angle6, c='green',label='$dp_7$')
# plt.ylabel('angle/rad')
# plt.xlabel('steps')
# plt.legend()
# plt.show()


plt.figure(figsize=(12, 8))
plt.rcParams.update({'font.size':12})

plt.subplot(4,2,1)
plt.plot(ep[:,0], color=(241/255, 64/255, 64/255),label='$error_{SCGP1}$')
plt.plot(ep_pd[:,0], color=(26/255, 111/255, 223/255),label='$error_{PD1}$')
plt.plot(error_std, c='gray', linestyle='--')
plt.ylabel('error/rad')
# plt.xlabel('steps')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='upper right')

plt.subplot(4,2,2)
plt.plot(ep[:,1], color=(241/255, 64/255, 64/255),label='$error_{SCGP2}$')
plt.plot(ep_pd[:,1], color=(26/255, 111/255, 223/255),label='$error_{PD2}$')
plt.plot(error_std, c='gray', linestyle='--')
# plt.ylabel('error/rad')
# plt.xlabel('steps')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='upper right')

plt.subplot(4,2,3)
plt.plot(ep[:,2], color=(241/255, 64/255, 64/255),label='$error_{SCGP3}$')
plt.plot(ep_pd[:,2], color=(26/255, 111/255, 223/255),label='$error_{PD3}$')
plt.plot(error_std, c='gray', linestyle='--')
plt.ylabel('error/rad')
# plt.xlabel('steps')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='upper right')

plt.subplot(4,2,4)
plt.plot(ep[:,3], color=(241/255, 64/255, 64/255),label='$error_{SCGP4}$')
plt.plot(ep_pd[:,3], color=(26/255, 111/255, 223/255),label='$error_{PD4}$')
plt.plot(error_std, c='gray', linestyle='--')
# plt.ylabel('error/rad')
# plt.xlabel('steps')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='upper right')

plt.subplot(4,2,5)
plt.plot(ep[:,4], color=(241/255, 64/255, 64/255),label='$error_{SCGP5}$')
plt.plot(ep_pd[:,4], color=(26/255, 111/255, 223/255),label='$error_{PD5}$')
plt.plot(error_std, c='gray', linestyle='--')
plt.ylabel('error/rad')
plt.xlabel('steps') 
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='upper right')

plt.subplot(4,2,6)
plt.plot(ep[:,5], color=(241/255, 64/255, 64/255),label='$error_{SCGP6}$')
plt.plot(ep_pd[:,5], color=(26/255, 111/255, 223/255),label='$error_{PD6}$')
plt.plot(error_std, c='gray', linestyle='--')
# plt.ylabel('error/rad')
plt.xlabel('steps')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='upper right')

plt.subplot(4,2,7)
plt.plot(ep[:,6], color=(241/255, 64/255, 64/255),label='$error_{SCGP7}$')
plt.plot(ep_pd[:,6], color=(26/255, 111/255, 223/255),label='$error_{PD7}$')
plt.plot(error_std, c='gray', linestyle='--')
plt.ylabel('error/rad')
plt.xlabel('steps')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='upper right')
plt.subplots_adjust(left=0.125, bottom=0.125, right=0.9, top=0.9, wspace=0.2, hspace=0.35)
# plt.savefig("fig999999999.png", dpi=800)
plt.show()
