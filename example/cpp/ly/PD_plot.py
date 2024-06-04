import numpy as np
import matplotlib.pyplot as plt

t = np.arange(0, 10, 0.01)
gap = 10
delta_angle0 = np.pi / 50.0 * (1 - np.cos(np.pi / 4 * 1 * t))
delta_angle1 = np.pi / 50.0 * (1 - np.cos(np.pi / 4 * 3 * t)) + np.pi / 6
delta_angle2 = np.pi / 50.0 * (1 - np.cos(np.pi / 2 * 1 * t))
delta_angle3 = np.pi / 70.0 * (1 - np.cos(np.pi * t)) + np.pi / 3
delta_angle4 = np.pi / 60.0 * (1 - np.cos(np.pi * t))
delta_angle5 = np.pi / 60.0 * (1 - np.cos(np.pi / 2 * 4 * t)) + np.pi / 2
delta_angle6 = np.pi / 70.0 * (1 - np.cos(np.pi / 2 * 3 * t))
index_8 = int(8/0.01)
delta_angle0[index_8:] = 0
delta_angle1[index_8:] = np.pi / 6
delta_angle2[index_8:] = 0
delta_angle3[index_8:] = np.pi / 3
delta_angle4[index_8:] = 0
delta_angle5[index_8:] = np.pi / 2
delta_angle6[index_8:] = 0

dp = np.hstack((delta_angle0, delta_angle1, delta_angle2, delta_angle3, delta_angle4, delta_angle5, delta_angle6))
# p = np.loadtxt('/home/robot/robot/roake_param_identify/build/PD_error/position_ly.txt')[0::gap,:]
# ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/PD_error/position_error_ly.txt')[0::gap,:]

p = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_1/position_ly.txt')[0::gap,:]
ep = np.loadtxt('/home/robot/robot/roake_param_identify/build/simulation_data_1/position_error_ly.txt')[0::gap,:]
plt.subplot(4,2,1)
plt.plot(p[:,0], c='red',label='$p_1$')
plt.plot(delta_angle0, c='green',label='$dp_1$')
plt.plot(ep[:,0], c='blue',label='$ep_1$')
plt.legend()
plt.subplot(4,2,2)
plt.plot(p[:,1], c='red',label='$p_2$')
plt.plot(delta_angle1, c='green',label='$dp_2$')
plt.plot(ep[:,1], c='blue',label='$ep_2$')
plt.legend()
plt.subplot(4,2,3)
plt.plot(p[:,2], c='red',label='$p_3$')
plt.plot(delta_angle2, c='green',label='$dp_3$')
plt.plot(ep[:,2], c='blue',label='$ep_3$')
plt.legend()
plt.subplot(4,2,4)
plt.plot(p[:,3], c='red',label='$p_4$')
plt.plot(delta_angle3, c='green',label='$dp_4$')
plt.plot(ep[:,3], c='blue',label='$ep_4$')
plt.legend()
plt.subplot(4,2,5)
plt.plot(p[:,4], c='red',label='$p_5$')
plt.plot(delta_angle4, c='green',label='$dp_5$')
plt.plot(ep[:,4], c='blue',label='$ep_5$')
plt.legend()
plt.subplot(4,2,6)
plt.plot(p[:,5], c='red',label='$p_6$')
plt.plot(delta_angle5, c='green',label='$dp_6$')
plt.plot(ep[:,5], c='blue',label='$ep_6$')
plt.legend()
plt.subplot(4,2,7)
plt.plot(p[:,6], c='red',label='$p_7$')
plt.plot(delta_angle6, c='green',label='$dp_7$')
plt.plot(ep[:,6], c='blue',label='$ep_7$')
plt.legend()
plt.show()