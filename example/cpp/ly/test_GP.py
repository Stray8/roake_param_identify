import numpy as np
import matplotlib.pyplot as plt
from GP_fitting import sgpr

np.random.seed(5)
trainingSet = np.loadtxt('/home/robot/robot/roake_param_identify/build/PD_error/trainingSet_ly.txt')
print('the training size is ', np.shape(trainingSet)[0])
Robot_DoF = 7
X, Y = trainingSet[:, 0:(3 * Robot_DoF)], trainingSet[:, (3 * Robot_DoF):]
y0 = Y[:, 0]
y1 = Y[:, 1]
y2 = Y[:, 2]
y3 = Y[:, 3]
y4 = Y[:, 4]
y5 = Y[:, 5]
y6 = Y[:, 6]

likelihood_noise = 0.5

gpr_fitter = sgpr(X, y0, likelihood_noise=likelihood_noise)
print('--- 0Start training ---')
gpr_fitter.train()
print('--- 0Training success---')

hat_y0, _ = gpr_fitter.predict_determined_input(X)
hat_y0 = hat_y0.reshape(-1)

gpr_fitter = sgpr(X, y1, likelihood_noise=likelihood_noise)
print('--- 1Start training ---')
gpr_fitter.train()
print('--- 1Training success---')

hat_y1, _ = gpr_fitter.predict_determined_input(X)
hat_y1 = hat_y1.reshape(-1)

gpr_fitter = sgpr(X, y2, likelihood_noise=likelihood_noise)
print('--- 2Start training ---')
gpr_fitter.train()
print('--- 2Training success---')

hat_y2, _ = gpr_fitter.predict_determined_input(X)
hat_y2 = hat_y2.reshape(-1)

gpr_fitter = sgpr(X, y3, likelihood_noise=likelihood_noise)
print('--- 3Start training ---')
gpr_fitter.train()
print('--- 3Training success---')

hat_y3, _ = gpr_fitter.predict_determined_input(X)
hat_y3 = hat_y3.reshape(-1)

gpr_fitter = sgpr(X, y4, likelihood_noise=likelihood_noise)
print('--- 4Start training ---')
gpr_fitter.train()
print('--- 4Training success---')

hat_y4, _ = gpr_fitter.predict_determined_input(X)
hat_y4 = hat_y4.reshape(-1)

gpr_fitter = sgpr(X, y5, likelihood_noise=likelihood_noise)
print('--- 5Start training ---')
gpr_fitter.train()
print('--- 5Training success---')

hat_y5, _ = gpr_fitter.predict_determined_input(X)
hat_y5 = hat_y5.reshape(-1)

gpr_fitter = sgpr(X, y6, likelihood_noise=likelihood_noise)
print('--- 6Start training ---')
gpr_fitter.train()
print('--- 6Training success---')

hat_y6, _ = gpr_fitter.predict_determined_input(X)
hat_y6 = hat_y6.reshape(-1)

data_Size = np.shape(trainingSet)[0]
print('--- Results on training Set ---')
# 绘制第1个子图：y0和hat_y0
plt.subplot(421)
plt.plot(np.arange(data_Size), y0, c='red', label='$tau_1$')
plt.plot(np.arange(data_Size), hat_y0, c='blue', label='$\hat tau_1$')
plt.legend()
# 绘制第2个子图：y1和hat_y1
plt.subplot(422)
plt.plot(np.arange(data_Size), y1, c='red', label='$tau_2$')
plt.plot(np.arange(data_Size), hat_y1, c='blue', label='$\hat tau_2$')
plt.legend()
# 绘制第3个子图：y2和hat_y2
plt.subplot(423)
plt.plot(np.arange(data_Size), y2, c='red', label='$tau_3$')
plt.plot(np.arange(data_Size), hat_y2, c='blue', label='$\hat tau_3$')
plt.legend()
# 绘制第4个子图：y3和hat_y3
plt.subplot(424)
plt.plot(np.arange(data_Size), y3, c='red', label='$tau_4$')
plt.plot(np.arange(data_Size), hat_y3, c='blue', label='$\hat tau_4$')
plt.legend()
# 绘制第5个子图：y4和hat_y4
plt.subplot(425)
plt.plot(np.arange(data_Size), y4, c='red', label='$tau_5$')
plt.plot(np.arange(data_Size), hat_y4, c='blue', label='$\hat tau_5$')
plt.legend()
# 绘制第6个子图：y5和hat_y5
plt.subplot(426)
plt.plot(np.arange(data_Size), y5, c='red', label='$tau_6$')
plt.plot(np.arange(data_Size), hat_y5, c='blue', label='$\hat tau_6$')
plt.legend()
# 绘制第7个子图：y6和hat_y6
plt.subplot(427)
plt.plot(np.arange(data_Size), y6, c='red', label='$tau_7$')
plt.plot(np.arange(data_Size), hat_y6, c='blue', label='$\hat tau_7$')
plt.legend()
# 显示图形
plt.show()