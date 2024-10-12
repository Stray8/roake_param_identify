import numpy as np
import matplotlib.pyplot as plt
import math

t = np.arange(0, 10, 0.01)
gap = 10

p = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/drag_data/position.txt')
dp = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/drag_data/velocity.txt')
# ddp = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/drag_data/acceleration.txt')
# p[:,6] = 0
# dp[:,6] = 0
# ddp[:,6] = 0

alpha = 0.5
N = np.shape(p)[0]
hat_p = np.zeros((N - 1, 7))
hat_dp = np.zeros((N - 1, 7))
hat_ddp = np.zeros((N - 1, 7))

for i in range(N - 2): # i是从0开始的
    hat_p[i+1,:] = alpha * p[i,:] + (1 - alpha) * p[i+1,:]
    hat_dp[i+1,:] = alpha * dp[i,:] + (1 - alpha) * dp[i+1,:]

hat_ddq = (hat_dp[1:, :] - hat_dp[0:-1, :]) / (gap * 1e-3)

    # hat_ddp[i+1,:] = alpha * ddp[i,:] + (1 - alpha) * ddp[i+1,:]

# np.savetxt('/home/robot/robot/roake_param_identify/build/drag/drag_data/hat_p.txt', p)
# np.savetxt('/home/robot/robot/roake_param_identify/build/drag/drag_data/hat_dp.txt', hat_dp)
# np.savetxt('/home/robot/robot/roake_param_identify/build/drag/drag_data/hat_ddp.txt', hat_ddp)


# position
plt.subplot(4,2,1)
plt.plot(p[:,0], c='green',label='$p_1$')
plt.plot(hat_p[:,0], c='blue',label='$hatp_1$')
plt.legend()
plt.subplot(4,2,2)
plt.plot(p[:,1], c='green',label='$p_2$')
plt.plot(hat_p[:,1], c='blue',label='$hatp_2$')
plt.legend()
plt.subplot(4,2,3)
plt.plot(p[:,2], c='green',label='$p_3$')
plt.plot(hat_p[:,2], c='blue',label='$hatp_3$')
plt.legend()
plt.subplot(4,2,4)
plt.plot(p[:,3], c='green',label='$p_4$')
plt.plot(hat_p[:,3], c='blue',label='$hatp_4$')
plt.legend()
plt.subplot(4,2,5)
plt.plot(p[:,4], c='green',label='$p_5$')
plt.plot(hat_p[:,4], c='blue',label='$hatp_5$')
plt.legend()
plt.subplot(4,2,6)
plt.plot(p[:,5], c='green',label='$p_6$')
plt.plot(hat_p[:,5], c='blue',label='$hatp_6$')
plt.legend()
plt.subplot(4,2,7)
plt.plot(p[:,6], c='green',label='$p_7$')
plt.plot(hat_p[:,6], c='blue',label='$hatp_7$')
plt.legend()
plt.show()
# velocity
plt.subplot(4,2,1)
plt.plot(dp[:,0], c='green',label='$dp_1$')
plt.plot(hat_dp[:,0], c='blue',label='$hatdp_1$')
plt.legend()
plt.subplot(4,2,2)
plt.plot(dp[:,1], c='green',label='$dp_2$')
plt.plot(hat_dp[:,1], c='blue',label='$hatdp_2$')
plt.legend()
plt.subplot(4,2,3)
plt.plot(dp[:,2], c='green',label='$dp_3$')
plt.plot(hat_dp[:,2], c='blue',label='$hatdp_3$')
plt.legend()
plt.subplot(4,2,4)
plt.plot(dp[:,3], c='green',label='$dp_4$')
plt.plot(hat_dp[:,3], c='blue',label='$hatdp_4$')
plt.legend()
plt.subplot(4,2,5)
plt.plot(dp[:,4], c='green',label='$dp_5$')
plt.plot(hat_dp[:,4], c='blue',label='$hatdp_5$')
plt.legend()
plt.subplot(4,2,6)
plt.plot(dp[:,5], c='green',label='$dp_6$')
plt.plot(hat_dp[:,5], c='blue',label='$hatdp_6$')
plt.legend()
plt.subplot(4,2,7)
plt.plot(dp[:,6], c='green',label='$dp_7$')
plt.plot(hat_dp[:,6], c='blue',label='$hatdp_7$')
plt.legend()
plt.show()
# velocity
plt.subplot(4,2,1)
# plt.plot(ddp[:,0], c='green',label='$ddp_1$')
plt.plot(hat_ddp[:,0], c='blue',label='$hatddp_1$')
plt.legend()
plt.subplot(4,2,2)
# plt.plot(ddp[:,1], c='green',label='$ddp_2$')
plt.plot(hat_ddp[:,1], c='blue',label='$hatddp_2$')
plt.legend()
plt.subplot(4,2,3)
# plt.plot(ddp[:,2], c='green',label='$ddp_3$')
plt.plot(hat_ddp[:,2], c='blue',label='$hatddp_3$')
plt.legend()
plt.subplot(4,2,4)
# plt.plot(ddp[:,3], c='green',label='$ddp_4$')
plt.plot(hat_ddp[:,3], c='blue',label='$hatddp_4$')
plt.legend()
plt.subplot(4,2,5)
# plt.plot(ddp[:,4], c='green',label='$ddp_5$')
plt.plot(hat_ddp[:,4], c='blue',label='$hatddp_5$')
plt.legend()
plt.subplot(4,2,6)
# plt.plot(ddp[:,5], c='green',label='$ddp_6$')
plt.plot(hat_ddp[:,5], c='blue',label='$hatddp_6$')
plt.legend()
plt.subplot(4,2,7)
# plt.plot(ddp[:,6], c='green',label='$ddp_7$')
plt.plot(hat_ddp[:,6], c='blue',label='$hatddp_7$')
plt.legend()
plt.show()