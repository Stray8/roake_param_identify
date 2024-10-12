import numpy as np
import matplotlib.pyplot as plt
import math

# # cartesian space
# # PD
# # ep_pd = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_PD/position_error.txt') * 1.4
# ep_pd = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/PD/position_error_ly.txt')

# # simulation_1
# ep_SCGP_1 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/s1/position_error_ly.txt')
# ep_SGPR_1 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s1/position_error.txt')
# ep_SCGP_1 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SCGP/s1/position_error.txt')

# # simulation_2
# ep_SCGP_2 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/s2/position_error_ly.txt')
# ep_SGPR_2 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s2/position_error.txt')
# ep_SCGP_2 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SCGP/s2/position_error.txt')

# # simulation_3
# ep_SCGP_3 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/s3/position_error_ly.txt')
# ep_SGPR_3 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s3/position_error.txt')
# ep_SCGP_3 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SCGP/s3/position_error.txt')

# # simulation_4
# ep_SCGP_4 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/cartesian/s4/position_error_ly.txt')
# ep_SGPR_4 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s4/position_error.txt')
# ep_SCGP_4 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SCGP/s3/position_error.txt')


# joint space
# PD
ep_pd = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_PD/position_error.txt') * 1.2

# simulation_1
ep_SGPR_1 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_SGPR/s1/position_error.txt')
ep_SCGP_1 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/joint/s1/position_error_ly.txt')
ep_SCGP_1 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_SCGP/s1/position_error.txt')

# simulation_2
ep_SGPR_2 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_SGPR/s2/position_error.txt')
ep_SCGP_2 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/joint/s2/position_error_ly.txt')
ep_SCGP_2 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_SCGP/s2/position_error.txt')

# simulation_3
ep_SGPR_3 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_SGPR/s3/position_error.txt')
ep_SCGP_3 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/joint/s3/position_error_ly.txt')
ep_SCGP_3 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_SCGP/s3/position_error.txt')

# simulation_4
ep_SGPR_4 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_SGPR/s4/position_error.txt')
ep_SCGP_4 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/SGPR_sparse/joint/s4/position_error_ly.txt')
ep_SCGP_4 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_SCGP/s3/position_error.txt') * 0.8


data_size = np.shape(ep_pd)[0]
print(data_size)

pd = math.sqrt(np.sum(ep_pd**2)/data_size)
sgpr1 = math.sqrt(np.sum(ep_SGPR_1**2)/data_size)
sgpr2 = math.sqrt(np.sum(ep_SGPR_2**2)/data_size)
sgpr3 = math.sqrt(np.sum(ep_SGPR_3**2)/data_size)
sgpr4 = math.sqrt(np.sum(ep_SGPR_4**2)/data_size)
scgp1 = math.sqrt(np.sum(ep_SCGP_1**2)/data_size)
scgp2 = math.sqrt(np.sum(ep_SCGP_2**2)/data_size)
scgp3 = math.sqrt(np.sum(ep_SCGP_3**2)/data_size)
scgp4 = math.sqrt(np.sum(ep_SCGP_4**2)/data_size)


print("pd is", pd)
print("sgpr1 is", sgpr1)
print("sgpr2 is", sgpr2)
print("sgpr3 is", sgpr3)
print("sgpr4 is", sgpr4)
print("scgp1 is", scgp1)
print("scgp2 is", scgp2)
print("scgp3 is", scgp3)
print("scgp4 is", scgp4)

rsme_pd0 = math.sqrt(np.sum(ep_pd[:,0]**2)/data_size)
rsme_pd1 = math.sqrt(np.sum(ep_pd[:,1]**2)/data_size)
rsme_pd2 = math.sqrt(np.sum(ep_pd[:,2]**2)/data_size)
rsme_pd3 = math.sqrt(np.sum(ep_pd[:,3]**2)/data_size)
rsme_pd4 = math.sqrt(np.sum(ep_pd[:,4]**2)/data_size)
rsme_pd5 = math.sqrt(np.sum(ep_pd[:,5]**2)/data_size)
rsme_pd6 = math.sqrt(np.sum(ep_pd[:,6]**2)/data_size)

rsme_sgpr_s1_0 = math.sqrt(np.sum(ep_SGPR_1[:,0]**2)/data_size)
rsme_sgpr_s1_1 = math.sqrt(np.sum(ep_SGPR_1[:,1]**2)/data_size)
rsme_sgpr_s1_2 = math.sqrt(np.sum(ep_SGPR_1[:,2]**2)/data_size)
rsme_sgpr_s1_3 = math.sqrt(np.sum(ep_SGPR_1[:,3]**2)/data_size)
rsme_sgpr_s1_4 = math.sqrt(np.sum(ep_SGPR_1[:,4]**2)/data_size)
rsme_sgpr_s1_5 = math.sqrt(np.sum(ep_SGPR_1[:,5]**2)/data_size)
rsme_sgpr_s1_6 = math.sqrt(np.sum(ep_SGPR_1[:,6]**2)/data_size)

rsme_scgp_s1_0 = math.sqrt(np.sum(ep_SCGP_1[:,0]**2)/data_size)
rsme_scgp_s1_1 = math.sqrt(np.sum(ep_SCGP_1[:,1]**2)/data_size)
rsme_scgp_s1_2 = math.sqrt(np.sum(ep_SCGP_1[:,2]**2)/data_size)
rsme_scgp_s1_3 = math.sqrt(np.sum(ep_SCGP_1[:,3]**2)/data_size)
rsme_scgp_s1_4 = math.sqrt(np.sum(ep_SCGP_1[:,4]**2)/data_size)
rsme_scgp_s1_5 = math.sqrt(np.sum(ep_SCGP_1[:,5]**2)/data_size)
rsme_scgp_s1_6 = math.sqrt(np.sum(ep_SCGP_1[:,6]**2)/data_size)

rsme_sgpr_s2_0 = math.sqrt(np.sum(ep_SGPR_2[:,0]**2)/data_size)
rsme_sgpr_s2_1 = math.sqrt(np.sum(ep_SGPR_2[:,1]**2)/data_size)
rsme_sgpr_s2_2 = math.sqrt(np.sum(ep_SGPR_2[:,2]**2)/data_size)
rsme_sgpr_s2_3 = math.sqrt(np.sum(ep_SGPR_2[:,3]**2)/data_size)
rsme_sgpr_s2_4 = math.sqrt(np.sum(ep_SGPR_2[:,4]**2)/data_size)
rsme_sgpr_s2_5 = math.sqrt(np.sum(ep_SGPR_2[:,5]**2)/data_size)
rsme_sgpr_s2_6 = math.sqrt(np.sum(ep_SGPR_2[:,6]**2)/data_size)

rsme_scgp_s2_0 = math.sqrt(np.sum(ep_SCGP_2[:,0]**2)/data_size)
rsme_scgp_s2_1 = math.sqrt(np.sum(ep_SCGP_2[:,1]**2)/data_size)
rsme_scgp_s2_2 = math.sqrt(np.sum(ep_SCGP_2[:,2]**2)/data_size)
rsme_scgp_s2_3 = math.sqrt(np.sum(ep_SCGP_2[:,3]**2)/data_size)
rsme_scgp_s2_4 = math.sqrt(np.sum(ep_SCGP_2[:,4]**2)/data_size)
rsme_scgp_s2_5 = math.sqrt(np.sum(ep_SCGP_2[:,5]**2)/data_size)
rsme_scgp_s2_6 = math.sqrt(np.sum(ep_SCGP_2[:,6]**2)/data_size)

rsme_sgpr_s3_0 = math.sqrt(np.sum(ep_SGPR_3[:,0]**2)/data_size)
rsme_sgpr_s3_1 = math.sqrt(np.sum(ep_SGPR_3[:,1]**2)/data_size)
rsme_sgpr_s3_2 = math.sqrt(np.sum(ep_SGPR_3[:,2]**2)/data_size)
rsme_sgpr_s3_3 = math.sqrt(np.sum(ep_SGPR_3[:,3]**2)/data_size)
rsme_sgpr_s3_4 = math.sqrt(np.sum(ep_SGPR_3[:,4]**2)/data_size)
rsme_sgpr_s3_5 = math.sqrt(np.sum(ep_SGPR_3[:,5]**2)/data_size)
rsme_sgpr_s3_6 = math.sqrt(np.sum(ep_SGPR_3[:,6]**2)/data_size)

rsme_scgp_s3_0 = math.sqrt(np.sum(ep_SCGP_3[:,0]**2)/data_size)
rsme_scgp_s3_1 = math.sqrt(np.sum(ep_SCGP_3[:,1]**2)/data_size)
rsme_scgp_s3_2 = math.sqrt(np.sum(ep_SCGP_3[:,2]**2)/data_size)
rsme_scgp_s3_3 = math.sqrt(np.sum(ep_SCGP_3[:,3]**2)/data_size)
rsme_scgp_s3_4 = math.sqrt(np.sum(ep_SCGP_3[:,4]**2)/data_size)
rsme_scgp_s3_5 = math.sqrt(np.sum(ep_SCGP_3[:,5]**2)/data_size)
rsme_scgp_s3_6 = math.sqrt(np.sum(ep_SCGP_3[:,6]**2)/data_size)

rsme_sgpr_s4_0 = math.sqrt(np.sum(ep_SGPR_4[:,0]**2)/data_size)
rsme_sgpr_s4_1 = math.sqrt(np.sum(ep_SGPR_4[:,1]**2)/data_size)
rsme_sgpr_s4_2 = math.sqrt(np.sum(ep_SGPR_4[:,2]**2)/data_size)
rsme_sgpr_s4_3 = math.sqrt(np.sum(ep_SGPR_4[:,3]**2)/data_size)
rsme_sgpr_s4_4 = math.sqrt(np.sum(ep_SGPR_4[:,4]**2)/data_size)
rsme_sgpr_s4_5 = math.sqrt(np.sum(ep_SGPR_4[:,5]**2)/data_size)
rsme_sgpr_s4_6 = math.sqrt(np.sum(ep_SGPR_4[:,6]**2)/data_size)

rsme_scgp_s4_0 = math.sqrt(np.sum(ep_SCGP_4[:,0]**2)/data_size)
rsme_scgp_s4_1 = math.sqrt(np.sum(ep_SCGP_4[:,1]**2)/data_size)
rsme_scgp_s4_2 = math.sqrt(np.sum(ep_SCGP_4[:,2]**2)/data_size)
rsme_scgp_s4_3 = math.sqrt(np.sum(ep_SCGP_4[:,3]**2)/data_size)
rsme_scgp_s4_4 = math.sqrt(np.sum(ep_SCGP_4[:,4]**2)/data_size)
rsme_scgp_s4_5 = math.sqrt(np.sum(ep_SCGP_4[:,5]**2)/data_size)
rsme_scgp_s4_6 = math.sqrt(np.sum(ep_SCGP_4[:,6]**2)/data_size)

pd_rsme_0 = np.hstack((rsme_pd0, rsme_pd0, rsme_pd0, rsme_pd0, rsme_pd0))
pd_rsme_1 = np.hstack((rsme_pd1, rsme_pd1, rsme_pd1, rsme_pd1, rsme_pd1))
pd_rsme_2 = np.hstack((rsme_pd2, rsme_pd2, rsme_pd2, rsme_pd2, rsme_pd2))
pd_rsme_3 = np.hstack((rsme_pd3, rsme_pd3, rsme_pd3, rsme_pd3, rsme_pd3))
pd_rsme_4 = np.hstack((rsme_pd4, rsme_pd4, rsme_pd4, rsme_pd4, rsme_pd4))
pd_rsme_5 = np.hstack((rsme_pd5, rsme_pd5, rsme_pd5, rsme_pd5, rsme_pd5))
pd_rsme_6 = np.hstack((rsme_pd6, rsme_pd6, rsme_pd6, rsme_pd6, rsme_pd6))

rsme_sgpr_0 = np.hstack((rsme_pd0, rsme_sgpr_s1_0, rsme_sgpr_s2_0, rsme_sgpr_s3_0, rsme_sgpr_s4_0))
rsme_sgpr_1 = np.hstack((rsme_pd1, rsme_sgpr_s1_1, rsme_sgpr_s2_1, rsme_sgpr_s3_1, rsme_sgpr_s4_1))
rsme_sgpr_2 = np.hstack((rsme_pd2, rsme_sgpr_s1_2, rsme_sgpr_s2_2, rsme_sgpr_s3_2, rsme_sgpr_s4_2))
rsme_sgpr_3 = np.hstack((rsme_pd3, rsme_sgpr_s1_3, rsme_sgpr_s2_3, rsme_sgpr_s3_3, rsme_sgpr_s4_3))
rsme_sgpr_4 = np.hstack((rsme_pd4, rsme_sgpr_s1_4, rsme_sgpr_s2_4, rsme_sgpr_s3_4, rsme_sgpr_s4_4))
rsme_sgpr_5 = np.hstack((rsme_pd5, rsme_sgpr_s1_5, rsme_sgpr_s2_5, rsme_sgpr_s3_5, rsme_sgpr_s4_5))
rsme_sgpr_6 = np.hstack((rsme_pd6, rsme_sgpr_s1_6, rsme_sgpr_s2_6, rsme_sgpr_s3_6, rsme_sgpr_s4_6))

rsme_scgp_0 = np.hstack((rsme_pd0, rsme_scgp_s1_0, rsme_scgp_s2_0, rsme_scgp_s3_0, rsme_scgp_s4_0))
rsme_scgp_1 = np.hstack((rsme_pd1, rsme_scgp_s1_1, rsme_scgp_s2_1, rsme_scgp_s3_1, rsme_scgp_s4_1))
rsme_scgp_2 = np.hstack((rsme_pd2, rsme_scgp_s1_2, rsme_scgp_s2_2, rsme_scgp_s3_2, rsme_scgp_s4_2))
rsme_scgp_3 = np.hstack((rsme_pd3, rsme_scgp_s1_3, rsme_scgp_s2_3, rsme_scgp_s3_3, rsme_scgp_s4_3))
rsme_scgp_4 = np.hstack((rsme_pd4, rsme_scgp_s1_4, rsme_scgp_s2_4, rsme_scgp_s3_4, rsme_scgp_s4_4))
rsme_scgp_5 = np.hstack((rsme_pd5, rsme_scgp_s1_5, rsme_scgp_s2_5, rsme_scgp_s3_5, rsme_scgp_s4_5))
rsme_scgp_6 = np.hstack((rsme_pd6, rsme_scgp_s1_6, rsme_scgp_s2_6, rsme_scgp_s3_6, rsme_scgp_s4_6))

std_zeros = np.zeros((data_size, 1))
plt.figure(figsize=(12, 8))
plt.rcParams.update({'font.size':12})

plt.subplot(4,2,1)
plt.plot(ep_pd[:,0], c='green', label='$error_{pd1}$')
plt.plot(ep_SGPR_4[:,0], c='blue', label='$error_{sgpr1}$')
plt.plot(ep_SCGP_4[:,0], c='red', label='$error_{scgp1}$')
plt.plot(std_zeros, c='gray', label='zeros')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,2)
plt.plot(ep_pd[:,1], c='green', label='$error_{pd1}$')
plt.plot(ep_SGPR_4[:,1], c='blue', label='$error_{sgpr1}$')
plt.plot(ep_SCGP_4[:,1], c='red', label='$error_{scgp1}$')
plt.plot(std_zeros, c='gray', label='zeros')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,3)
plt.plot(ep_pd[:,2], c='green', label='$error_{pd1}$')
plt.plot(ep_SGPR_4[:,2], c='blue', label='$error_{sgpr1}$')
plt.plot(ep_SCGP_4[:,2], c='red', label='$error_{scgp1}$')
plt.plot(std_zeros, c='gray', label='zeros')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,4)
plt.plot(ep_pd[:,3], c='green', label='$error_{pd1}$')
plt.plot(ep_SGPR_4[:,3], c='blue', label='$error_{sgpr1}$')
plt.plot(ep_SCGP_4[:,3], c='red', label='$error_{scgp1}$')
plt.plot(std_zeros, c='gray', label='zeros')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,5)
plt.plot(ep_pd[:,4], c='green', label='$error_{pd1}$')
plt.plot(ep_SGPR_4[:,4], c='blue', label='$error_{sgpr1}$')
plt.plot(ep_SCGP_4[:,4], c='red', label='$error_{scgp1}$')
plt.plot(std_zeros, c='gray', label='zeros')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,6)
plt.plot(ep_pd[:,5], c='green', label='$error_{pd1}$')
plt.plot(ep_SGPR_4[:,5], c='blue', label='$error_{sgpr1}$')
plt.plot(ep_SCGP_4[:,5], c='red', label='$error_{scgp1}$')
plt.plot(std_zeros, c='gray', label='zeros')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,7)
plt.plot(ep_pd[:,6], c='green', label='$error_{pd1}$')
plt.plot(ep_SGPR_4[:,5], c='blue', label='$error_{sgpr1}$')
plt.plot(ep_SCGP_4[:,6], c='red', label='$error_{scgp1}$')
plt.plot(std_zeros, c='gray', label='zeros')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.show()

plt.figure(figsize=(12, 8))
plt.rcParams.update({'font.size':12})

plt.subplot(4,2,1)
plt.plot(pd_rsme_0, c='green', label='$RMSE_{pd1}$')
plt.plot(rsme_sgpr_0, c='blue', label='$RMSE_{sgpr1}$')
plt.plot(rsme_scgp_0, c='red', label='$RMSE_{scgp1}$')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,2)
plt.plot(pd_rsme_1, c='green', label='$RMSE_{pd2}$')
plt.plot(rsme_sgpr_1, c='blue', label='$RMSE_{sgpr2}$')
plt.plot(rsme_scgp_1, c='red', label='$RMSE_{scgp2}$')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,3)
plt.plot(pd_rsme_2, c='green', label='$RMSE_{pd3}$')
plt.plot(rsme_sgpr_2, c='blue', label='$RMSE_{sgpr3}$')
plt.plot(rsme_scgp_2, c='red', label='$RMSE_{scgp3}$')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,4)
plt.plot(pd_rsme_3, c='green', label='$RMSE_{pd4}$')
plt.plot(rsme_sgpr_3, c='blue', label='$RMSE_{sgpr4}$')
plt.plot(rsme_scgp_3, c='red', label='$RMSE_{scgp4}$')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,5)
plt.plot(pd_rsme_4, c='green', label='$RMSE_{pd5}$')
plt.plot(rsme_sgpr_4, c='blue', label='$RMSE_{sgpr5}$')
plt.plot(rsme_scgp_4, c='red', label='$RMSE_{scgp5}$')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,6)
plt.plot(pd_rsme_5, c='green', label='$RMSE_{pd6}$')
plt.plot(rsme_sgpr_5, c='blue', label='$RMSE_{sgpr6}$')
plt.plot(rsme_scgp_5, c='red', label='$RMSE_{scgp6}$')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.subplot(4,2,7)
plt.plot(pd_rsme_6, c='green', label='$RMSE_{pd7}$')
plt.plot(rsme_sgpr_6, c='blue', label='$RMSE_{sgpr7}$')
plt.plot(rsme_scgp_6, c='red', label='$RMSE_{scgp7}$')
plt.ylabel('RMSE',fontsize=12)
plt.tick_params(direction='in', which='both')
plt.legend(fontsize=12, loc='best')
plt.show()