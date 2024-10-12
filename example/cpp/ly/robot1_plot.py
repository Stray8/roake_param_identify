import numpy as np
import matplotlib.pyplot as plt
import math

t = np.arange(0, 10, 0.01)
gap = 100

p_d = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/drag_data/position.txt')[0::gap,:]
dp_d = np.loadtxt('/home/robot/robot/roake_param_identify/build/drag/drag_data/velocity.txt')[0::gap,:]


plt.subplot(4,2,1)
plt.plot(p_d[:,0], c='red',label='$p_1$')
plt.plot(dp_d[:,0], c='blue',label='$dp_1$')
plt.legend()
plt.subplot(4,2,2)
plt.plot(p_d[:,1], c='red',label='$p_2$')
plt.plot(dp_d[:,1], c='blue',label='$dp_2$')

plt.legend()
plt.subplot(4,2,3)
plt.plot(p_d[:,2], c='red',label='$p_3$')
plt.plot(dp_d[:,2], c='blue',label='$dp_3$')

plt.legend()
plt.subplot(4,2,4)
plt.plot(p_d[:,3], c='red',label='$p_4$')
plt.plot(dp_d[:,3], c='blue',label='$dp_4$')

plt.legend()
plt.subplot(4,2,5)
plt.plot(p_d[:,4], c='red',label='$p_5$')
plt.plot(dp_d[:,4], c='blue',label='$dp_5$')

plt.legend()
plt.subplot(4,2,6)
plt.plot(p_d[:,5], c='red',label='$p_6$')
plt.plot(dp_d[:,5], c='blue',label='$dp_6$')

plt.legend()
plt.subplot(4,2,7)
plt.plot(p_d[:,6], c='red',label='$p_7$')
plt.plot(dp_d[:,6], c='blue',label='$dp_7$')

plt.legend()
plt.show()