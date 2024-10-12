import numpy as np
import matplotlib.pyplot as plt
import math

gap = 10
# PD
ep_pd = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/PD_error/position_error_ly_PD.txt')[0::gap,:]
data_size = np.shape(ep_pd)[0]
rsme_pd0 = math.sqrt(np.sum(ep_pd[:,0]**2)/data_size)
rsme_pd1 = math.sqrt(np.sum(ep_pd[:,1]**2)/data_size)
rsme_pd2 = math.sqrt(np.sum(ep_pd[:,2]**2)/data_size)
rsme_pd3 = math.sqrt(np.sum(ep_pd[:,3]**2)/data_size)
rsme_pd4 = math.sqrt(np.sum(ep_pd[:,4]**2)/data_size)
rsme_pd5 = math.sqrt(np.sum(ep_pd[:,5]**2)/data_size)
rsme_pd6 = math.sqrt(np.sum(ep_pd[:,6]**2)/data_size)
mae_pd0 = np.sum(abs(ep_pd[:,0]))/data_size
mae_pd1 = np.sum(abs(ep_pd[:,1]))/data_size
mae_pd2 = np.sum(abs(ep_pd[:,2]))/data_size
mae_pd3 = np.sum(abs(ep_pd[:,3]))/data_size
mae_pd4 = np.sum(abs(ep_pd[:,4]))/data_size
mae_pd5 = np.sum(abs(ep_pd[:,5]))/data_size
mae_pd6 = np.sum(abs(ep_pd[:,6]))/data_size

# simulation_1
ep_simulation_1 = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/simulation_1/position_error_ly_s1.txt')[0::gap,:]
rsme_simulation_1_0 = math.sqrt(np.sum(ep_simulation_1[:,0]**2)/data_size)
rsme_simulation_1_1 = math.sqrt(np.sum(ep_simulation_1[:,1]**2)/data_size)
rsme_simulation_1_2 = math.sqrt(np.sum(ep_simulation_1[:,2]**2)/data_size)
rsme_simulation_1_3 = math.sqrt(np.sum(ep_simulation_1[:,3]**2)/data_size)
rsme_simulation_1_4 = math.sqrt(np.sum(ep_simulation_1[:,4]**2)/data_size)
rsme_simulation_1_5 = math.sqrt(np.sum(ep_simulation_1[:,5]**2)/data_size)
rsme_simulation_1_6 = math.sqrt(np.sum(ep_simulation_1[:,6]**2)/data_size)
mae_simulation_1_0 = np.sum(abs(ep_simulation_1[:,0]))/data_size
mae_simulation_1_1 = np.sum(abs(ep_simulation_1[:,1]))/data_size
mae_simulation_1_2 = np.sum(abs(ep_simulation_1[:,2]))/data_size
mae_simulation_1_3 = np.sum(abs(ep_simulation_1[:,3]))/data_size
mae_simulation_1_4 = np.sum(abs(ep_simulation_1[:,4]))/data_size
mae_simulation_1_5 = np.sum(abs(ep_simulation_1[:,5]))/data_size
mae_simulation_1_6 = np.sum(abs(ep_simulation_1[:,6]))/data_size

# simulation_2
ep_simulation_2 = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/simulation_2/position_error_ly.txt')[0::gap,:]
rsme_simulation_2_0 = math.sqrt(np.sum(ep_simulation_2[:,0]**2)/data_size)
rsme_simulation_2_1 = math.sqrt(np.sum(ep_simulation_2[:,1]**2)/data_size)
rsme_simulation_2_2 = math.sqrt(np.sum(ep_simulation_2[:,2]**2)/data_size)
rsme_simulation_2_3 = math.sqrt(np.sum(ep_simulation_2[:,3]**2)/data_size)
rsme_simulation_2_4 = math.sqrt(np.sum(ep_simulation_2[:,4]**2)/data_size)
rsme_simulation_2_5 = math.sqrt(np.sum(ep_simulation_2[:,5]**2)/data_size)
rsme_simulation_2_6 = math.sqrt(np.sum(ep_simulation_2[:,6]**2)/data_size)
mae_simulation_2_0 = np.sum(abs(ep_simulation_2[:,0]))/data_size
mae_simulation_2_1 = np.sum(abs(ep_simulation_2[:,1]))/data_size
mae_simulation_2_2 = np.sum(abs(ep_simulation_2[:,2]))/data_size
mae_simulation_2_3 = np.sum(abs(ep_simulation_2[:,3]))/data_size
mae_simulation_2_4 = np.sum(abs(ep_simulation_2[:,4]))/data_size
mae_simulation_2_5 = np.sum(abs(ep_simulation_2[:,5]))/data_size
mae_simulation_2_6 = np.sum(abs(ep_simulation_2[:,6]))/data_size
# simulation_3
ep_simulation_3 = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/simulation_3/position_error_ly.txt')[0::gap,:]
rsme_simulation_3_0 = math.sqrt(np.sum(ep_simulation_3[:,0]**2)/data_size)
rsme_simulation_3_1 = math.sqrt(np.sum(ep_simulation_3[:,1]**2)/data_size)
rsme_simulation_3_2 = math.sqrt(np.sum(ep_simulation_3[:,2]**2)/data_size)
rsme_simulation_3_3 = math.sqrt(np.sum(ep_simulation_3[:,3]**2)/data_size)
rsme_simulation_3_4 = math.sqrt(np.sum(ep_simulation_3[:,4]**2)/data_size)
rsme_simulation_3_5 = math.sqrt(np.sum(ep_simulation_3[:,5]**2)/data_size)
rsme_simulation_3_6 = math.sqrt(np.sum(ep_simulation_3[:,6]**2)/data_size)
mae_simulation_3_0 = np.sum(abs(ep_simulation_3[:,0]))/data_size
mae_simulation_3_1 = np.sum(abs(ep_simulation_3[:,1]))/data_size
mae_simulation_3_2 = np.sum(abs(ep_simulation_3[:,2]))/data_size
mae_simulation_3_3 = np.sum(abs(ep_simulation_3[:,3]))/data_size
mae_simulation_3_4 = np.sum(abs(ep_simulation_3[:,4]))/data_size
mae_simulation_3_5 = np.sum(abs(ep_simulation_3[:,5]))/data_size
mae_simulation_3_6 = np.sum(abs(ep_simulation_3[:,6]))/data_size
# simulation_4
ep_simulation_4 = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/simulation_4/position_error_ly.txt')[0::gap,:]
rsme_simulation_4_0 = math.sqrt(np.sum(ep_simulation_4[:,0]**2)/data_size)
rsme_simulation_4_1 = math.sqrt(np.sum(ep_simulation_4[:,1]**2)/data_size)
rsme_simulation_4_2 = math.sqrt(np.sum(ep_simulation_4[:,2]**2)/data_size)
rsme_simulation_4_3 = math.sqrt(np.sum(ep_simulation_4[:,3]**2)/data_size)
rsme_simulation_4_4 = math.sqrt(np.sum(ep_simulation_4[:,4]**2)/data_size)
rsme_simulation_4_5 = math.sqrt(np.sum(ep_simulation_4[:,5]**2)/data_size)
rsme_simulation_4_6 = math.sqrt(np.sum(ep_simulation_4[:,6]**2)/data_size)
mae_simulation_4_0 = np.sum(abs(ep_simulation_4[:,0]))/data_size
mae_simulation_4_1 = np.sum(abs(ep_simulation_4[:,1]))/data_size
mae_simulation_4_2 = np.sum(abs(ep_simulation_4[:,2]))/data_size
mae_simulation_4_3 = np.sum(abs(ep_simulation_4[:,3]))/data_size
mae_simulation_4_4 = np.sum(abs(ep_simulation_4[:,4]))/data_size
mae_simulation_4_5 = np.sum(abs(ep_simulation_4[:,5]))/data_size
mae_simulation_4_6 = np.sum(abs(ep_simulation_4[:,6]))/data_size
# simulation_5
ep_simulation_5 = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/simulation_5/position_error_ly.txt')[0::gap,:]
rsme_simulation_5_0 = math.sqrt(np.sum(ep_simulation_5[:,0]**2)/data_size)
rsme_simulation_5_1 = math.sqrt(np.sum(ep_simulation_5[:,1]**2)/data_size)
rsme_simulation_5_2 = math.sqrt(np.sum(ep_simulation_5[:,2]**2)/data_size)
rsme_simulation_5_3 = math.sqrt(np.sum(ep_simulation_5[:,3]**2)/data_size)
rsme_simulation_5_4 = math.sqrt(np.sum(ep_simulation_5[:,4]**2)/data_size)
rsme_simulation_5_5 = math.sqrt(np.sum(ep_simulation_5[:,5]**2)/data_size)
rsme_simulation_5_6 = math.sqrt(np.sum(ep_simulation_5[:,6]**2)/data_size)
mae_simulation_5_0 = np.sum(abs(ep_simulation_5[:,0]))/data_size
mae_simulation_5_1 = np.sum(abs(ep_simulation_5[:,1]))/data_size
mae_simulation_5_2 = np.sum(abs(ep_simulation_5[:,2]))/data_size
mae_simulation_5_3 = np.sum(abs(ep_simulation_5[:,3]))/data_size
mae_simulation_5_4 = np.sum(abs(ep_simulation_5[:,4]))/data_size
mae_simulation_5_5 = np.sum(abs(ep_simulation_5[:,5]))/data_size
mae_simulation_5_6 = np.sum(abs(ep_simulation_5[:,6]))/data_size
# simulation_6
ep_simulation_6 = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/simulation_6/position_error_ly.txt')[0::gap,:]
rsme_simulation_6_0 = math.sqrt(np.sum(ep_simulation_6[:,0]**2)/data_size)
rsme_simulation_6_1 = math.sqrt(np.sum(ep_simulation_6[:,1]**2)/data_size)
rsme_simulation_6_2 = math.sqrt(np.sum(ep_simulation_6[:,2]**2)/data_size)
rsme_simulation_6_3 = math.sqrt(np.sum(ep_simulation_6[:,3]**2)/data_size)
rsme_simulation_6_4 = math.sqrt(np.sum(ep_simulation_6[:,4]**2)/data_size)
rsme_simulation_6_5 = math.sqrt(np.sum(ep_simulation_6[:,5]**2)/data_size)
rsme_simulation_6_6 = math.sqrt(np.sum(ep_simulation_6[:,6]**2)/data_size)
mae_simulation_6_0 = np.sum(abs(ep_simulation_6[:,0]))/data_size
mae_simulation_6_1 = np.sum(abs(ep_simulation_6[:,1]))/data_size
mae_simulation_6_2 = np.sum(abs(ep_simulation_6[:,2]))/data_size
mae_simulation_6_3 = np.sum(abs(ep_simulation_6[:,3]))/data_size
mae_simulation_6_4 = np.sum(abs(ep_simulation_6[:,4]))/data_size
mae_simulation_6_5 = np.sum(abs(ep_simulation_6[:,5]))/data_size
mae_simulation_6_6 = np.sum(abs(ep_simulation_6[:,6]))/data_size

rsme_0 = np.hstack((rsme_pd0, rsme_simulation_1_0, rsme_simulation_2_0, rsme_simulation_3_0, rsme_simulation_4_0,rsme_simulation_5_0,rsme_simulation_6_0))
rsme_1 = np.hstack((rsme_pd1, rsme_simulation_1_1, rsme_simulation_2_1, rsme_simulation_3_1, rsme_simulation_4_1,rsme_simulation_5_1,rsme_simulation_6_1))
rsme_2 = np.hstack((rsme_pd2, rsme_simulation_1_2, rsme_simulation_2_2, rsme_simulation_3_2, rsme_simulation_4_2,rsme_simulation_5_2,rsme_simulation_6_2))
rsme_3 = np.hstack((rsme_pd3, rsme_simulation_1_3, rsme_simulation_2_3, rsme_simulation_3_3, rsme_simulation_4_3,rsme_simulation_5_3,rsme_simulation_6_3))
rsme_4 = np.hstack((rsme_pd4, rsme_simulation_1_4, rsme_simulation_2_4, rsme_simulation_3_4, rsme_simulation_4_4,rsme_simulation_5_4,rsme_simulation_6_4))
rsme_5 = np.hstack((rsme_pd5, rsme_simulation_1_5, rsme_simulation_2_5, rsme_simulation_3_5, rsme_simulation_4_5,rsme_simulation_5_5,rsme_simulation_6_5))
rsme_6 = np.hstack((rsme_pd6, rsme_simulation_1_6, rsme_simulation_2_6, rsme_simulation_3_6, rsme_simulation_4_6,rsme_simulation_5_6,rsme_simulation_6_6))
mae_0 = np.hstack((mae_pd0, mae_simulation_1_0, mae_simulation_2_0, mae_simulation_3_0, mae_simulation_4_0,mae_simulation_5_0,mae_simulation_6_0))
mae_1 = np.hstack((mae_pd1, mae_simulation_1_1, mae_simulation_2_1, mae_simulation_3_1, mae_simulation_4_1,mae_simulation_5_1,mae_simulation_6_1))
mae_2 = np.hstack((mae_pd2, mae_simulation_1_2, mae_simulation_2_2, mae_simulation_3_2, mae_simulation_4_2,mae_simulation_5_2,mae_simulation_6_2))
mae_3 = np.hstack((mae_pd3, mae_simulation_1_3, mae_simulation_2_3, mae_simulation_3_3, mae_simulation_4_3,mae_simulation_5_3,mae_simulation_6_3))
mae_4 = np.hstack((mae_pd4, mae_simulation_1_4, mae_simulation_2_4, mae_simulation_3_4, mae_simulation_4_4,mae_simulation_5_4,mae_simulation_6_4))
mae_5 = np.hstack((mae_pd5, mae_simulation_1_5, mae_simulation_2_5, mae_simulation_3_5, mae_simulation_4_5,mae_simulation_5_5,mae_simulation_6_5))
mae_6 = np.hstack((mae_pd6, mae_simulation_1_6, mae_simulation_2_6, mae_simulation_3_6, mae_simulation_4_6,mae_simulation_5_6,mae_simulation_6_6))

pd_rsme_0 = np.hstack((rsme_pd0, rsme_pd0, rsme_pd0, rsme_pd0, rsme_pd0,rsme_pd0,rsme_pd0))
pd_rsme_1 = np.hstack((rsme_pd1, rsme_pd1, rsme_pd1, rsme_pd1, rsme_pd1,rsme_pd1,rsme_pd1))
pd_rsme_2 = np.hstack((rsme_pd2, rsme_pd2, rsme_pd2, rsme_pd2, rsme_pd2,rsme_pd2,rsme_pd2))
pd_rsme_3 = np.hstack((rsme_pd3, rsme_pd3, rsme_pd3, rsme_pd3, rsme_pd3,rsme_pd3,rsme_pd3))
pd_rsme_4 = np.hstack((rsme_pd4, rsme_pd4, rsme_pd4, rsme_pd4, rsme_pd4,rsme_pd4,rsme_pd4))
pd_rsme_5 = np.hstack((rsme_pd5, rsme_pd5, rsme_pd5, rsme_pd5, rsme_pd5,rsme_pd5,rsme_pd5))
pd_rsme_6 = np.hstack((rsme_pd6, rsme_pd6, rsme_pd6, rsme_pd6, rsme_pd6,rsme_pd6,rsme_pd6))


# plt.subplot(4,2,1)
# plt.plot(rsme_0, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme0}$')
# plt.plot(pd_rsme_0, color=(26/255, 111/255, 223/255),label='$PD_{rsme0}$')
# # plt.plot(mae_0, c='blue',label='$mae_0$')
# plt.ylabel('rsme')
# # plt.xlabel('times')
# plt.legend()
# plt.subplot(4,2,2)
# plt.plot(rsme_1, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme1}$')
# plt.plot(pd_rsme_1, color=(26/255, 111/255, 223/255),label='$PD_{rsme1}$')

# # plt.plot(mae_1, c='blue',label='$mae_1$')
# # plt.ylabel('rsme')
# # plt.xlabel('times')
# plt.legend()
# plt.subplot(4,2,3)
# plt.plot(rsme_2, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme2}$')
# plt.plot(pd_rsme_2, color=(26/255, 111/255, 223/255),label='$PD_{rsme2}$')

# # plt.plot(mae_2, c='blue',label='$mae_2$')
# plt.ylabel('rsme')
# # plt.xlabel('times')
# plt.legend()
# plt.subplot(4,2,4)
# plt.plot(rsme_3, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme3}$')
# plt.plot(pd_rsme_3, color=(26/255, 111/255, 223/255),label='$PD_{rsme3}$')

# # plt.plot(mae_3, c='blue',label='$mae_3$')
# # plt.ylabel('rsme')
# # plt.xlabel('times')
# plt.legend()
# plt.subplot(4,2,5)
# plt.plot(rsme_4, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme4}$')
# plt.plot(pd_rsme_4, color=(26/255, 111/255, 223/255),label='$PD_{rsme4}$')

# # plt.plot(mae_4, c='blue',label='$mae_4$')
# plt.ylabel('rsme')
# # plt.xlabel('times')
# plt.legend()
# plt.subplot(4,2,6)
# plt.plot(rsme_5, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme5}$')
# plt.plot(pd_rsme_5, color=(26/255, 111/255, 223/255),label='$PD_{rsme5}$')

# # plt.plot(mae_5, c='blue',label='$mae_5$')
# # plt.ylabel('rsme')
# plt.xlabel('times')
# plt.legend()
# plt.subplot(4,2,7)
# plt.plot(rsme_6, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme6}$')
# plt.plot(pd_rsme_6, color=(26/255, 111/255, 223/255),label='$PD_{rsme6}$')

# # plt.plot(mae_6, c='blue',label='$mae_6$')
# plt.ylabel('rsme')
# plt.xlabel('times')
# plt.legend()
# plt.show()
# plt.savefig("fig7", dpi=300, format="svg")



plt.figure(figsize=(12, 8))
plt.rcParams.update({'font.size':12})

plt.subplot(4,2,1)
plt.plot(rsme_0, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme0}$')
plt.plot(pd_rsme_0, color=(26/255, 111/255, 223/255),label='$PD_{rsme0}$')
# plt.plot(mae_0, c='blue',label='$mae_0$')
plt.ylabel('rsme',fontsize=12)
# plt.xlabel('times')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,2)
plt.plot(rsme_1, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme1}$')
plt.plot(pd_rsme_1, color=(26/255, 111/255, 223/255),label='$PD_{rsme1}$')
# plt.plot(mae_1, c='blue',label='$mae_1$')
# plt.ylabel('rsme')
# plt.xlabel('times')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,3)
plt.plot(rsme_2, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme2}$')
plt.plot(pd_rsme_2, color=(26/255, 111/255, 223/255),label='$PD_{rsme2}$')
# plt.plot(mae_2, c='blue',label='$mae_2$')
plt.ylabel('rsme',fontsize=12)
# plt.xlabel('times')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,4)
plt.plot(rsme_3, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme3}$')
plt.plot(pd_rsme_3, color=(26/255, 111/255, 223/255),label='$PD_{rsme3}$')
# plt.plot(mae_3, c='blue',label='$mae_3$')
# plt.ylabel('rsme')
# plt.xlabel('times')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,5)
plt.plot(rsme_4, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme4}$')
plt.plot(pd_rsme_4, color=(26/255, 111/255, 223/255),label='$PD_{rsme4}$')
# plt.plot(mae_4, c='blue',label='$mae_4$')
plt.ylabel('rsme',fontsize=12)
# plt.xlabel('times')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,6)
plt.plot(rsme_5, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme5}$')
plt.plot(pd_rsme_5, color=(26/255, 111/255, 223/255),label='$PD_{rsme5}$')
# plt.plot(mae_5, c='blue',label='$mae_5$')
# plt.ylabel('rsme')
plt.xlabel('times')
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplot(4,2,7)
plt.plot(rsme_6, color=(241/255, 64/255, 64/255),label='$SGPR_{rsme6}$')
plt.plot(pd_rsme_6, color=(26/255, 111/255, 223/255),label='$PD_{rsme6}$')
# plt.plot(mae_6, c='blue',label='$mae_6$')
plt.ylabel('rsme',fontsize=12)
plt.xlabel('times',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')

plt.subplots_adjust(left=0.125, bottom=0.125, right=0.9, top=0.9, wspace=0.2, hspace=0.35)
# plt.subplots_adjust(left=0.125, bottom=0.1, right=0.9, to;p=0.9, wspace=0.2, hspace=0.35)
# plt.savefig('fig7.png', dpi=800)
plt.show()
