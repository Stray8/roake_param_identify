'''
Learning the original ADS using Gaussian Process.
Details can be found in the paper:
"Learning a flexible neural energy function with a unique minimum
for stable and accurate demonstration learning"
'''

import autograd.numpy as np
from autograd import value_and_grad
from scipy.optimize import minimize
import autograd.scipy.stats.multivariate_normal as mvn
from autograd.numpy.linalg import solve
import matplotlib as plt
import matplotlib.pyplot as plt

np.random.seed(5)

class SingleGpr:
    def __init__(self, X, y, observation_noise=0.2, gamma=1):
        '''
        Initializing the single GPR
        :param X: Input set, (input_num * input_dim)
        :param y: Output set for one dim, (n_size,)
        :param observation_noise: the standard deviation for observation_noise
        :param gamma: a scalar which will be used if observation_noise is None
        '''
        self.input_dim = np.shape(X)[1]
        # add the equilibrium-point (0, 0) to the set
        self.X = np.vstack((X, np.zeros(self.input_dim).reshape(1, -1)))
        self.y = np.hstack((y, np.array([0.0])))
        self.input_num = np.shape(self.X)[0]
        if observation_noise is None:
            self.observation_noise = gamma * np.sqrt(np.average(self.y**2))
        else:
            self.observation_noise = observation_noise
        self.param = self.init_random_param()
        # Used to tune the degree of the equilibrium-point confidence
        self.determined_point_degree = 0.0

    def init_random_param(self):
        '''
        Initializing the hyper-parameters
        '''
        sqrt_kernel_length_scale = np.sqrt(np.diag(np.cov(self.X.T)))
        kernel_noise = np.sqrt(np.average(self.y**2))
        param = np.hstack((kernel_noise, sqrt_kernel_length_scale))
        return param

    def set_param(self, param):
        '''
        Manually set the hyper-parameters
        '''
        self.param = param.copy()
        '''
        pre-computations for prediction
        '''
        self.cov_y_y = self.rbf(self.X, self.X, self.param)
        temp = self.observation_noise ** 2 * np.eye(self.input_num)
        # observation noises for determined set should be zero
        temp[-1, -1] = self.determined_point_degree
        self.cov_y_y = self.cov_y_y + temp
        self.beta = solve(self.cov_y_y, self.y)  # The constant vector of the mean prediction function
        self.inv_cov_y_y = solve(self.cov_y_y, np.eye(self.input_num))

    def build_objective(self, param):
        '''
        The obj of Single GPR
        '''
        cov_y_y = self.rbf(self.X, self.X, param)
        temp = self.observation_noise**2 * np.eye(self.input_num)
        # observation noises for determined set should be zero
        temp[-1, -1] = self.determined_point_degree
        cov_y_y = cov_y_y + temp
        out = - mvn.logpdf(self.y, np.zeros(self.input_num), cov_y_y)
        return out

    def train(self):
        '''
        Training Single GPR
        '''
        result = minimize(value_and_grad(self.build_objective), self.param, jac=True, method='L-BFGS-B', tol=1e-8,
                          options={'maxiter': 50, 'disp': False})
        self.param = result.x
        # pre-computation for prediction
        self.cov_y_y = self.rbf(self.X, self.X, self.param)
        temp = self.observation_noise ** 2 * np.eye(self.input_num)
        temp[-1, -1] = self.determined_point_degree
        self.cov_y_y = self.cov_y_y + temp
        self.beta = solve(self.cov_y_y, self.y)
        self.inv_cov_y_y = solve(self.cov_y_y, np.eye(self.input_num))

    def rbf(self, x, x_, param):
        '''
        Construct the kernel matrix (or scalar, vector)
        '''
        kn = param[0]  # abbreviation for kernel_noise
        sqrt_kls = param[1:]  # abbreviation for sqrt_kernel_length_scale
        '''
        Using the broadcast technique to accelerate computation
        '''
        diffs = np.expand_dims(x / sqrt_kls, 1) - np.expand_dims(x_ / sqrt_kls, 0)
        return kn ** 2 * np.exp(-0.5 * np.sum(diffs ** 2, axis=2))

    def predict_determined_input(self, inputs):
        '''
        Prediction of Single GPR
        '''
        cov_y_f = self.rbf(self.X, inputs, self.param)
        means = np.dot(cov_y_f.T, self.beta)  # (m,)
        return means


Robot_DOF = 7
# # cartesian space
# trainingSet_collect = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_collect/trainingSet_drag.txt')[0::2,:]
# trainingSet_s1 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s1/trainingset.txt')[0::4,:]
# trainingSet_s2 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s2/trainingset.txt')[0::4,:]
# trainingSet_s3 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s3/trainingset.txt')[0::4,:]
# trainingSet_s4 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/cartesian/data_SGPR/s4/trainingset.txt')[0::5,:]

# trainingSet = trainingSet_collect
# trainingSet = np.vstack((trainingSet_collect, trainingSet_s1))
# trainingSet = np.vstack((trainingSet_collect, trainingSet_s1, trainingSet_s2))
# trainingSet = np.vstack((trainingSet_collect, trainingSet_s1, trainingSet_s2, trainingSet_s3))
# trainingSet = np.vstack((trainingSet_collect, trainingSet_s1, trainingSet_s2, trainingSet_s3, trainingSet_s4))

# joint space
trainingSet_collect = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_collect/trainingset.txt')[0::2,:]
trainingSet_s1 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_SGPR/s1/trainingset.txt')[0::2,:]
trainingSet_s2 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_SGPR/s2/trainingset.txt')[0::2,:]
trainingSet_s3 = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/compare_SCGP_SGPR/joint/data_SGPR/s3/trainingset.txt')[0::2,:]

trainingSet = trainingSet_collect
trainingSet = np.vstack((trainingSet_collect, trainingSet_s1))
trainingSet = np.vstack((trainingSet_collect, trainingSet_s1, trainingSet_s2))
trainingSet = np.vstack((trainingSet_collect, trainingSet_s1, trainingSet_s2, trainingSet_s3))



print("size_trainingSet is ", np.shape(trainingSet))
X, Y = trainingSet[:, 0:(3*Robot_DOF)], trainingSet[:, (3*Robot_DOF):]
y0 = Y[:, 0]
y1 = Y[:, 1]
y2 = Y[:, 2]
y3 = Y[:, 3]
y4 = Y[:, 4]
y5 = Y[:, 5]
y6 = Y[:, 6]
# np.savetxt('D:/工大/ly/code/compare_SCGP_SGPR/Y.txt', Y)
print('strat training ... ')
print('strat training gpr_fitting0 ... ')
gpr_fitting0 = SingleGpr(X, y0)
gpr_fitting0.train()
print('end training gpr_fitting0 ... ')
print('strat training gpr_fitting1 ... ')
gpr_fitting1 = SingleGpr(X, y1)
gpr_fitting1.train()
print('end training gpr_fitting1 ... ')
print('strat training gpr_fitting2 ... ')
gpr_fitting2 = SingleGpr(X, y2)
gpr_fitting2.train()
print('end training gpr_fitting2 ... ')
print('strat training gpr_fitting3 ... ')
gpr_fitting3 = SingleGpr(X, y3)
gpr_fitting3.train()
print('end training gpr_fitting3 ... ')
print('strat training gpr_fitting4 ... ')
gpr_fitting4 = SingleGpr(X, y4)
gpr_fitting4.train()
print('end training gpr_fitting4 ... ')
print('strat training gpr_fitting5 ... ')
gpr_fitting5 = SingleGpr(X, y5)
gpr_fitting5.train()
print('end training gpr_fitting5 ... ')
print('strat training gpr_fitting6 ... ')
gpr_fitting6 = SingleGpr(X, y6)
gpr_fitting6.train()
print('end training gpr_fitting6 ... ')
print('end training ... ')

def prediction(input):
    hat_y0 = gpr_fitting0.predict_determined_input(input)
    hat_y1 = gpr_fitting1.predict_determined_input(input)
    hat_y2 = gpr_fitting2.predict_determined_input(input)
    hat_y3 = gpr_fitting3.predict_determined_input(input)
    hat_y4 = gpr_fitting4.predict_determined_input(input)
    hat_y5 = gpr_fitting5.predict_determined_input(input)
    hat_y6 = gpr_fitting6.predict_determined_input(input)

    hat_y = np.vstack((hat_y0, hat_y1, hat_y2, hat_y3, hat_y4, hat_y5, hat_y6))
    hat_y = hat_y.T
    # hat_y = np.hstack((hat_y0, hat_y1, hat_y2, hat_y3, hat_y4, hat_y5, hat_y6))
    return hat_y

input = X
hat_y = prediction(input)
print("size_hat_y is ", np.shape(hat_y))
# np.savetxt('D:/工大/ly/code/compare_SCGP_SGPR/tau_SGPR.txt', hat_y)
hat_y0 = hat_y[:,0]
hat_y1 = hat_y[:,1]
hat_y2 = hat_y[:,2]
hat_y3 = hat_y[:,3]
hat_y4 = hat_y[:,4]
hat_y5 = hat_y[:,5]
hat_y6 = hat_y[:,6]

data_Size = np.shape(trainingSet)[0]
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