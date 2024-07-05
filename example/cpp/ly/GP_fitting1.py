import autograd.numpy as np
import autograd.numpy.random as npr
from autograd import value_and_grad, grad
from scipy.optimize import minimize
from autograd.misc.optimizers import adam
import matplotlib.pyplot as plt
import autograd.scipy.stats.multivariate_normal as mvn
from autograd.numpy.linalg import solve
import faulthandler

np.random.seed(5) #生成随机数
faulthandler.enable()

class sgpr:
    def __init__(self, X, y, likelihood_noise=0.05, restart=1):
        '''
        :param X: q, dq, ddq
        :param y: output for one dim
        :param likelihood_noise:
        :param restart:
        '''
        X = np.array(X)
        y = np.array(y)
        self.X = X
        self.y = y
        self.input_dim = np.shape(self.X)[1] # X的列数
        self.input_num = np.shape(self.X)[0] # X的行数
        self.q_dim = self.input_dim // 3 # 单纯q的列数是7列
        self.param = np.empty(self.q_dim * 4 + 2) # 建立一个空的数组
        self.likelihood_noise = likelihood_noise
        self.restart = restart
        self.cov_y_y = None # 初始化参数cov_y_y
        self.beta = None # 初始化参数beta

    def init_random_param(self): # 生成初始的训练参数
        # np.random.normal 函数生成一个符合正态分布的随机数数组
        # size 指定了生成数组的大小
        # self.input_dim 可能是某个类的属性，表示输入的维度。// 是整数除法，返回的是整数部分
        sqrt_kls_m = 0.01 * np.random.normal(size=self.input_dim // 3 * 2) + 2 # 均值为2，标准差为0.01
        sqrt_kls_c = 0.01 * np.random.normal(size=self.input_dim // 3 * 2) + 2 # 均值为2，标准差为0.01
        kn_m = 0.1 * np.random.normal(size=1) # 均值为1，标准差为0.1
        kn_c = 0.1 * np.random.normal(size=1) # 均值为1，标准差为0.1
        self.param = np.hstack((kn_m, sqrt_kls_m, kn_c, sqrt_kls_c))
    
    # def save_param(self, /home/robot/robot/roake_param_identify/build/collect/parameter.txt):
        # np.savetxt('/home/robot/robot/roake_param_identify/build/collect/parameter.txt', self.param)
        
    def set_param(self, param):
        self.param = param.copy()
        # 使用rbf核函数，再加上对角阵（表示噪声的方差）
        self.cov_y_y = self.rbf(self.X, self.X, self.param) + self.likelihood_noise ** 2 * np.eye(self.input_num)
        self.beta = solve(self.cov_y_y, self.y) # 计算beta
        self.inv_cov_y_y = solve(self.cov_y_y, np.eye(self.input_num)) # 计算cov_y_y的逆矩阵

    def build_objective(self, param):
        param = np.array(param)
        cov_y_y = self.rbf(self.X, self.X, param)
        cov_y_y = cov_y_y + self.likelihood_noise**2 * np.eye(self.input_num)
        out = - mvn.logpdf(self.y, np.zeros(self.input_num), cov_y_y) # mvn.logpdf 函数计算该分布下观测值的对数概率密度
        return out # 返回计算得到的负对数似然值

    def train(self):
        max_logpdf = -1e20
        # cons = con((0.001, 10))
        for i in range(self.restart):
            self.init_random_param()
            # result = minimize(value_and_grad(self.build_objective), self.param, jac=True, method='L-BFGS-B', tol=1)
            result = minimize(value_and_grad(self.build_objective), self.param, jac=True, method='L-BFGS-B', tol=0.01)
            logpdf = -result.fun
            param = result.x
            if logpdf > max_logpdf:
                self.param = param
                max_logpdf = logpdf
        # 提前计算，做预测时可用
        self.cov_y_y = self.rbf(self.X, self.X, self.param) + self.likelihood_noise**2 * np.eye(self.input_num)
        self.beta = solve(self.cov_y_y, self.y)
        self.inv_cov_y_y = solve(self.cov_y_y, np.eye(self.input_num))

    def rbf_single(self, x, x_, param):
        '''
        :param x: row element is (q, dq) or (q, ddq)
        :param x_:
        :param param:
        :return:
        '''
        x = np.array(x)
        x_ = np.array(x_)
        kn = param[0]
        sqrt_kls = param[1:]
        diffs = np.expand_dims(x / sqrt_kls, 1) - np.expand_dims(x_ / sqrt_kls, 0)
        temp1 = np.tanh(0.1 * np.sqrt(np.sum(x[:, self.q_dim:] ** 2, axis=1)).reshape(-1, 1))
        temp2 = np.tanh(0.1 * np.sqrt(np.sum(x_[:, self.q_dim:] ** 2, axis=1)).reshape(1, -1))
        return temp1 * (kn**2 * np.exp(-0.5 * np.sum(diffs ** 2, axis=2))) * temp2

    def rbf(self, x, x_, param):
        '''
        :param x: row element is (q, dq, ddq)
        :param x_:
        :param param:
        :return:
        '''
        x = np.array(x)
        x_ = np.array(x_)
        x_m = np.hstack((x[:, 0:self.q_dim], x[:, self.q_dim * 2:]))
        x__m = np.hstack((x_[:, 0:self.q_dim], x_[:, self.q_dim * 2:]))
        param_m = param[0: (1 + self.q_dim * 2)]
        rbf_m = self.rbf_single(x_m, x__m, param_m)

        x_c = x[:, 0:self.q_dim * 2]
        x__c = x_[:, 0:self.q_dim * 2]
        param_c = param[(1 + self.q_dim * 2):]
        rbf_c = self.rbf_single(x_c, x__c, param_c)
        return rbf_m + rbf_c

    def predict_determined_input(self, inputs):  # 单维GP预测
        inputs = np.array(inputs)
        cov_y_f = self.rbf(self.X, inputs, self.param)
        mean_outputs= np.dot(cov_y_f.T, self.beta.reshape((-1, 1)))
        # var = (self.param[0]**2 - np.diag(np.dot(np.dot(cov_y_f.T, self.inv_cov_y_y), cov_y_f))).reshape(-1, 1)
        return mean_outputs, None


Robot_DOF = 7
trainingSet_collect = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/collect/trainingSet_collect.txt')
# trainingSet_s1 = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_1/trainingSet_s1.txt')
# trainingSet_s2 = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_2/trainingSet_s2.txt')
# trainingSet_s3 = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_3/trainingSet_s3.txt')
# trainingSet_s4 = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_4/trainingSet_s4.txt')
# trainingSet_s5 = np.loadtxt('/home/robot/robot/roake_param_identify/build/test/simulation_data_5/trainingSet_s5.txt')

trainingSet = trainingSet_collect
# trainingSet = np.hstack((trainingSet_collect, trainingSet_s1))
# trainingSet = np.hstack((trainingSet_collect, trainingSet_s1, trainingSet_s2))
# trainingSet = np.hstack((trainingSet_collect, trainingSet_s1, trainingSet_s2, trainingSet_s3))
# trainingSet = np.hstack((trainingSet_collect, trainingSet_s1, trainingSet_s2, trainingSet_s3, trainingSet_s4))
# trainingSet = np.hstack((trainingSet_collect, trainingSet_s1, trainingSet_s2, trainingSet_s3, trainingSet_s4, trainingSet_s5))

X, Y = trainingSet[:, 0:(3*Robot_DOF)], trainingSet[:, (3*Robot_DOF):]
y0 = Y[:, 0]
y1 = Y[:, 1]
y2 = Y[:, 2]
y3 = Y[:, 3]
y4 = Y[:, 4]
y5 = Y[:, 5]
y6 = Y[:, 6]

print('strat training ... ')
gpr_fitting0 = sgpr(X, y0)
gpr_fitting0.train()
gpr_fitting1 = sgpr(X, y1)
gpr_fitting1.train()
gpr_fitting2 = sgpr(X, y2)
gpr_fitting2.train()
gpr_fitting3 = sgpr(X, y3)
gpr_fitting3.train()
gpr_fitting4 = sgpr(X, y4)
gpr_fitting4.train()
gpr_fitting5 = sgpr(X, y5)
gpr_fitting5.train()
gpr_fitting6 = sgpr(X, y6)
gpr_fitting6.train()
print('end training ... ')


def prediction(input):
    pre0,_ = gpr_fitting0.predict_determined_input(input)
    pre1,_ = gpr_fitting1.predict_determined_input(input)
    pre2,_ = gpr_fitting2.predict_determined_input(input)
    pre3,_ = gpr_fitting3.predict_determined_input(input)
    pre4,_ = gpr_fitting4.predict_determined_input(input)
    pre5,_ = gpr_fitting5.predict_determined_input(input)
    pre6,_ = gpr_fitting6.predict_determined_input(input)
    # pre0 = pre0.reshape(-1)
    # pre1 = pre1.reshape(-1)
    # pre2 = pre2.reshape(-1)
    # pre3 = pre3.reshape(-1)
    # pre4 = pre4.reshape(-1)
    # pre5 = pre5.reshape(-1)
    # pre6 = pre6.reshape(-1)
    pre = np.hstack((pre0, pre1, pre2, pre3, pre4, pre5, pre6))
    return pre

print(X[0, :])
input = X
pre = prediction(input)
hat_y0 = pre[:,0]
hat_y1 = pre[:,1]
hat_y2 = pre[:,2]
hat_y3 = pre[:,3]
hat_y4 = pre[:,4]
hat_y5 = pre[:,5]
hat_y6 = pre[:,6]

data_Size = np.shape(trainingSet)[0]
# print("pre is", pre)
# 绘制第1个子图：y0和hat_y0
plt.subplot(421)
plt.plot(np.arange(data_Size), y0, color=(26/255, 111/255, 223/255), label='$tau_1$')
plt.plot(np.arange(data_Size), hat_y0, color=(241/255, 64/255, 64/255), label='$\hat tau_1$')
plt.ylabel('$mu$')
# plt.xlabel('steps')
plt.legend()
# 绘制第2个子图：y1和hat_y1
plt.subplot(422)
plt.plot(np.arange(data_Size), y1, color=(26/255, 111/255, 223/255), label='$tau_2$')
plt.plot(np.arange(data_Size), hat_y1, color=(241/255, 64/255, 64/255), label='$\hat tau_2$')
plt.ylabel('$mu$')
# plt.xlabel('steps')
plt.legend()
# 绘制第3个子图：y2和hat_y2
plt.subplot(423)
plt.plot(np.arange(data_Size), y2, color=(26/255, 111/255, 223/255), label='$tau_3$')
plt.plot(np.arange(data_Size), hat_y2, color=(241/255, 64/255, 64/255), label='$\hat tau_3$')
plt.ylabel('$mu$')
# plt.xlabel('steps')
plt.legend()
# 绘制第4个子图：y3和hat_y3
plt.subplot(424)
plt.plot(np.arange(data_Size), y3, color=(26/255, 111/255, 223/255), label='$tau_4$')
plt.plot(np.arange(data_Size), hat_y3, color=(241/255, 64/255, 64/255), label='$\hat tau_4$')
plt.ylabel('$mu$')
# plt.xlabel('steps')
plt.legend()
# 绘制第5个子图：y4和hat_y4
plt.subplot(425)
plt.plot(np.arange(data_Size), y4, color=(26/255, 111/255, 223/255), label='$tau_5$')
plt.plot(np.arange(data_Size), hat_y4, color=(241/255, 64/255, 64/255), label='$\hat tau_5$')
plt.ylabel('$mu$')
# plt.xlabel('steps')
plt.legend()
# 绘制第6个子图：y5和hat_y5
plt.subplot(426)
plt.plot(np.arange(data_Size), y5, color=(26/255, 111/255, 223/255), label='$tau_6$')
plt.plot(np.arange(data_Size), hat_y5, color=(241/255, 64/255, 64/255), label='$\hat tau_6$')
plt.ylabel('$mu$')
plt.xlabel('steps')
plt.legend()
# 绘制第7个子图：y6和hat_y6
plt.subplot(427)
plt.plot(np.arange(data_Size), y6, color=(26/255, 111/255, 223/255), label='$tau_7$')
plt.plot(np.arange(data_Size), hat_y6, color=(241/255, 64/255, 64/255), label='$\hat tau_7$')
plt.ylabel('$mu$')
plt.xlabel('steps')
plt.legend()
# 显示图形
plt.show()