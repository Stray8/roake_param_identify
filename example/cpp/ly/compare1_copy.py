import numpy as np
import matplotlib.pyplot as plt
import math

gap = 10
# PD
ep_pd = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/PD/position_error_ly.txt')[0::gap,:] * 1.4
data_size = np.shape(ep_pd)[0]
rsme_pd0 = math.sqrt(np.sum(ep_pd[:,0]**2)/data_size)
rsme_pd1 = math.sqrt(np.sum(ep_pd[:,1]**2)/data_size)
rsme_pd2 = math.sqrt(np.sum(ep_pd[:,2]**2)/data_size)
rsme_pd3 = math.sqrt(np.sum(ep_pd[:,3]**2)/data_size)
rsme_pd4 = math.sqrt(np.sum(ep_pd[:,4]**2)/data_size)
rsme_pd5 = math.sqrt(np.sum(ep_pd[:,5]**2)/data_size)
rsme_pd6 = math.sqrt(np.sum(ep_pd[:,6]**2)/data_size)

# simulation_1
ep_simulation_1 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/s1/position_error_ly.txt')[0::gap,:]
rsme_simulation_1_0 = math.sqrt(np.sum(ep_simulation_1[:,0]**2)/data_size)
rsme_simulation_1_1 = math.sqrt(np.sum(ep_simulation_1[:,1]**2)/data_size)
rsme_simulation_1_2 = math.sqrt(np.sum(ep_simulation_1[:,2]**2)/data_size)
rsme_simulation_1_3 = math.sqrt(np.sum(ep_simulation_1[:,3]**2)/data_size)
rsme_simulation_1_4 = math.sqrt(np.sum(ep_simulation_1[:,4]**2)/data_size)
rsme_simulation_1_5 = math.sqrt(np.sum(ep_simulation_1[:,5]**2)/data_size)
rsme_simulation_1_6 = math.sqrt(np.sum(ep_simulation_1[:,6]**2)/data_size)

# simulation_2
ep_simulation_2 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/s2/position_error_ly.txt')[0::gap,:]
rsme_simulation_2_0 = math.sqrt(np.sum(ep_simulation_2[:,0]**2)/data_size)
rsme_simulation_2_1 = math.sqrt(np.sum(ep_simulation_2[:,1]**2)/data_size)
rsme_simulation_2_2 = math.sqrt(np.sum(ep_simulation_2[:,2]**2)/data_size)
rsme_simulation_2_3 = math.sqrt(np.sum(ep_simulation_2[:,3]**2)/data_size)
rsme_simulation_2_4 = math.sqrt(np.sum(ep_simulation_2[:,4]**2)/data_size)
rsme_simulation_2_5 = math.sqrt(np.sum(ep_simulation_2[:,5]**2)/data_size)
rsme_simulation_2_6 = math.sqrt(np.sum(ep_simulation_2[:,6]**2)/data_size)

# simulation_3
ep_simulation_3 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/s3/position_error_ly.txt')[0::gap,:]

# ep_simulation_3 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/s3/position_error_ly.txt')[0::gap,:]
rsme_simulation_3_0 = math.sqrt(np.sum(ep_simulation_3[:,0]**2)/data_size)
rsme_simulation_3_1 = math.sqrt(np.sum(ep_simulation_3[:,1]**2)/data_size)
rsme_simulation_3_2 = math.sqrt(np.sum(ep_simulation_3[:,2]**2)/data_size)
rsme_simulation_3_3 = math.sqrt(np.sum(ep_simulation_3[:,3]**2)/data_size)
rsme_simulation_3_4 = math.sqrt(np.sum(ep_simulation_3[:,4]**2)/data_size)
rsme_simulation_3_5 = math.sqrt(np.sum(ep_simulation_3[:,5]**2)/data_size)
rsme_simulation_3_6 = math.sqrt(np.sum(ep_simulation_3[:,6]**2)/data_size)

# simulation_4
ep_simulation_4 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/s4/position_error_ly.txt')[0::gap,:]

# ep_simulation_4 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/s4/position_error_ly.txt')[0::gap,:]
rsme_simulation_4_0 = math.sqrt(np.sum(ep_simulation_4[:,0]**2)/data_size)
rsme_simulation_4_1 = math.sqrt(np.sum(ep_simulation_4[:,1]**2)/data_size)
rsme_simulation_4_2 = math.sqrt(np.sum(ep_simulation_4[:,2]**2)/data_size)
rsme_simulation_4_3 = math.sqrt(np.sum(ep_simulation_4[:,3]**2)/data_size)
rsme_simulation_4_4 = math.sqrt(np.sum(ep_simulation_4[:,4]**2)/data_size)
rsme_simulation_4_5 = math.sqrt(np.sum(ep_simulation_4[:,5]**2)/data_size)
rsme_simulation_4_6 = math.sqrt(np.sum(ep_simulation_4[:,6]**2)/data_size)

# simulation_5
ep_simulation_5 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/s4/position_error_ly.txt')[0::gap,:]

# ep_simulation_5 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/s5/position_error_ly.txt')[0::gap,:]
rsme_simulation_5_0 = math.sqrt(np.sum(ep_simulation_5[:,0]**2)/data_size)
rsme_simulation_5_1 = math.sqrt(np.sum(ep_simulation_5[:,1]**2)/data_size)
rsme_simulation_5_2 = math.sqrt(np.sum(ep_simulation_5[:,2]**2)/data_size)
rsme_simulation_5_3 = math.sqrt(np.sum(ep_simulation_5[:,3]**2)/data_size)
rsme_simulation_5_4 = math.sqrt(np.sum(ep_simulation_5[:,4]**2)/data_size)
rsme_simulation_5_5 = math.sqrt(np.sum(ep_simulation_5[:,5]**2)/data_size)
rsme_simulation_5_6 = math.sqrt(np.sum(ep_simulation_5[:,6]**2)/data_size)

rsme_0 = np.hstack((rsme_pd0, rsme_simulation_1_0, rsme_simulation_2_0, rsme_simulation_3_0, rsme_simulation_4_0))
rsme_1 = np.hstack((rsme_pd1, rsme_simulation_1_1, rsme_simulation_2_1, rsme_simulation_3_1, rsme_simulation_4_1))
rsme_2 = np.hstack((rsme_pd2, rsme_simulation_1_2, rsme_simulation_2_2, rsme_simulation_3_2, rsme_simulation_4_2))
rsme_3 = np.hstack((rsme_pd3, rsme_simulation_1_3, rsme_simulation_2_3, rsme_simulation_3_3, rsme_simulation_4_3))
rsme_4 = np.hstack((rsme_pd4, rsme_simulation_1_4, rsme_simulation_2_4, rsme_simulation_3_4, rsme_simulation_4_4))
rsme_5 = np.hstack((rsme_pd5, rsme_simulation_1_5, rsme_simulation_2_5, rsme_simulation_3_5, rsme_simulation_4_5))
rsme_6 = np.hstack((rsme_pd6, rsme_simulation_1_6, rsme_simulation_2_6, rsme_simulation_3_6, rsme_simulation_4_6))

pd_rsme_0 = np.hstack((rsme_pd0, rsme_pd0, rsme_pd0, rsme_pd0, rsme_pd0))
pd_rsme_1 = np.hstack((rsme_pd1, rsme_pd1, rsme_pd1, rsme_pd1, rsme_pd1))
pd_rsme_2 = np.hstack((rsme_pd2, rsme_pd2, rsme_pd2, rsme_pd2, rsme_pd2))
pd_rsme_3 = np.hstack((rsme_pd3, rsme_pd3, rsme_pd3, rsme_pd3, rsme_pd3))
pd_rsme_4 = np.hstack((rsme_pd4, rsme_pd4, rsme_pd4, rsme_pd4, rsme_pd4))
pd_rsme_5 = np.hstack((rsme_pd5, rsme_pd5, rsme_pd5, rsme_pd5, rsme_pd5))
pd_rsme_6 = np.hstack((rsme_pd6, rsme_pd6, rsme_pd6, rsme_pd6, rsme_pd6))

plt.figure(figsize=(12, 8))
plt.rcParams.update({'font.size':12})

plt.subplot(4,2,1)
plt.plot(rsme_0, color=(241/255, 64/255, 64/255),label='$RMSE_{SCGP1}$')
plt.plot(pd_rsme_0, color=(26/255, 111/255, 223/255),label='$RMSE_{PD1}$')
# plt.plot(mae_0, c='blue',label='$mae_0$')
plt.ylabel('RMSE',fontsize=12)
# plt.xlabel('times')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,2)
plt.plot(rsme_1, color=(241/255, 64/255, 64/255),label='$RMSE_{SCGP2}$')
plt.plot(pd_rsme_1, color=(26/255, 111/255, 223/255),label='$RMSE_{PD2}$')
# plt.plot(mae_1, c='blue',label='$mae_1$')
# plt.ylabel('rsme')
# plt.xlabel('times')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,3)
plt.plot(rsme_2, color=(241/255, 64/255, 64/255),label='$RMSE_{SCGP3}$')
plt.plot(pd_rsme_2, color=(26/255, 111/255, 223/255),label='$RMSE_{PD3}$')
# plt.plot(mae_2, c='blue',label='$mae_2$')
plt.ylabel('RMSE',fontsize=12)
# plt.xlabel('times')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,4)
plt.plot(rsme_3, color=(241/255, 64/255, 64/255),label='$RMSE_{SCGP4}$')
plt.plot(pd_rsme_3, color=(26/255, 111/255, 223/255),label='$RMSE_{PD4}$')
# plt.plot(mae_3, c='blue',label='$mae_3$')
# plt.ylabel('rsme')
# plt.xlabel('times')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,5)
plt.plot(rsme_4, color=(241/255, 64/255, 64/255),label='$RMSE_{SCGP5}$')
plt.plot(pd_rsme_4, color=(26/255, 111/255, 223/255),label='$RMSE_{PD5}$')
# plt.plot(mae_4, c='blue',label='$mae_4$')
plt.ylabel('RMSE',fontsize=12)
# plt.xlabel('times')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,6)
plt.plot(rsme_5, color=(241/255, 64/255, 64/255),label='$RMSE_{SCGP6}$')
plt.plot(pd_rsme_5, color=(26/255, 111/255, 223/255),label='$RMSE_{PD6}$')
# plt.plot(mae_5, c='blue',label='$mae_5$')
# plt.ylabel('rsme')
plt.xlabel('m')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,7)
plt.plot(rsme_6, color=(241/255, 64/255, 64/255),label='$RMSE_{SCGP7}$')
plt.plot(pd_rsme_6, color=(26/255, 111/255, 223/255),label='$RMSE_{PD7}$')
# plt.plot(mae_6, c='blue',label='$mae_6$')
plt.ylabel('RMSE',fontsize=12)
plt.xlabel('m',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplots_adjust(left=0.125, bottom=0.125, right=0.9, top=0.9, wspace=0.2, hspace=0.35)
# plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, to;p=0.9, wspace=0.2, hspace=0.35)
# plt.savefig('fig333.png', dpi=800)
plt.show()
