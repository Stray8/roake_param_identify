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
    def __init__(self, X, y, likelihood_noise=0.5, restart=1):
        '''
        :param X: q, dq, ddq
        :param y: output for one dim
        :param likelihood_noise:
        :param restart:
        '''
        # print("X is", X)
        # print("y is", y)
        # print("*****************************************************************************")
        # print("*****************************************************************************")
        X = np.array(X)
        y = np.array(y)
        # print("X is", X)
        # print("y is", y)
        self.X = X
        self.y = y
        self.input_dim = np.shape(self.X)[1] # X的列数
        self.input_num = np.shape(self.X)[0] # X的行数
        # print("self.input_num is", self.input_num)
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
        # print("kn_c is", kn_c)
        self.param = np.hstack((kn_m, sqrt_kls_m, kn_c, sqrt_kls_c))
        # print("self.param is", self.param)
        

    def set_param(self, param):
        self.param = param.copy()
        # 使用rbf核函数，再加上对角阵（表示噪声的方差）
        self.cov_y_y = self.rbf(self.X, self.X, self.param) + self.likelihood_noise ** 2 * np.eye(self.input_num)
        self.beta = solve(self.cov_y_y, self.y) # 计算beta
        self.inv_cov_y_y = solve(self.cov_y_y, np.eye(self.input_num)) # 计算cov_y_y的逆矩阵
        # print("inv_cov_y_y is", inv_cov_y_y)

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
        # print("inv_cov_y_y is", self.inv_cov_y_y)

    def rbf_single(self, x, x_, param):
        '''
        :param x: row element is (q, dq) or (q, ddq)
        :param x_:
        :param param:
        :return:
        '''
        # print("x is", x)
        # print("x_ is", x_)
        # print("*****************************************************************************")
        # print("*****************************************************************************")
        x = np.array(x)
        x_ = np.array(x_)
        # print("x is", x)
        # print("x_ is", x_)
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
        # print("x is", x)
        # print("x_ is", x_)
        # print("*****************************************************************************")
        # print("*****************************************************************************")
        x = np.array(x)
        x_ = np.array(x_)
        # print("x is", x)
        # print("x_ is", x_)
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
        # inputs 是矩阵
        inputs = np.array(inputs)
        cov_y_f = self.rbf(self.X, inputs, self.param)
        # print("cov_y_f is", cov_y_f)
        # print("beta is", self.beta)
        # mean_outputs, _ = np.dot(cov_y_f.T, self.beta.reshape((-1, 1)))
        mean_outputs= np.dot(cov_y_f.T, self.beta.reshape((-1, 1)))
        # print("mean_outputs is", mean_outputs)
        # var = (self.param[0]**2 - np.diag(np.dot(np.dot(cov_y_f.T, self.inv_cov_y_y), cov_y_f))).reshape(-1, 1)
        return mean_outputs, None


# def prediction(x):
#     Robot_DOF = 7
#     trainingSet = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/trainingSet_ly.txt')
#     # trainingSet = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/trainingSet_ly1.txt')
#     # trainingSet = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/trainingSet_ly2.txt')
#     # trainingSet = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/trainingSet_ly3.txt')
#     # trainingSet = np.loadtxt('/home/robot/robot/roake_param_identify/example/cpp/ly/trainingSet_ly4.txt')

#     # print("the trainingSet size is", np.shape(trainingSet)[0])
#     X, Y = trainingSet[:, 0:(3*Robot_DOF)], trainingSet[:, (3*Robot_DOF):]
#     # X, Y = Z[:, 0:(3*Robot_DOF)], Z[:, (3*Robot_DOF):]

#     y0 = Y[:, 0]
#     y1 = Y[:, 1]
#     y2 = Y[:, 2]
#     y3 = Y[:, 3]
#     y4 = Y[:, 4]
#     y5 = Y[:, 5]
#     y6 = Y[:, 6]

#     # print("---start training pre0---")
#     gpr_fitting = sgpr(X, y0)
#     gpr_fitting.train()
#     pre0,_ = gpr_fitting.predict_determined_input(x)
#     pre0 = pre0.reshape(-1)
#     # pre0 = pre0 - y0
#     # print("---training pre0 success---")

#     # print("---start training pre1---")
#     gpr_fitting = sgpr(X, y1)
#     gpr_fitting.train()
#     pre1,_ = gpr_fitting.predict_determined_input(x)
#     pre1 = pre1.reshape(-1)
#     # pre1 = pre1 - y1
#     # print("---training pre1 success---")

#     # print("---start training pre2---")
#     gpr_fitting = sgpr(X, y2)
#     gpr_fitting.train()
#     pre2,_ = gpr_fitting.predict_determined_input(x)
#     pre2 = pre2.reshape(-1)
#     # pre2 = pre2 - y2
#     # print("---training pre2 success---")

#     # print("---start training pre3---")
#     gpr_fitting = sgpr(X, y3)
#     gpr_fitting.train()
#     pre3,_ = gpr_fitting.predict_determined_input(x)
#     pre3 = pre3.reshape(-1)
#     # pre3 = pre3 - y3
#     # print("---training pre3 success---")

#     # print("---start training pre4---")
#     gpr_fitting = sgpr(X, y4)
#     gpr_fitting.train()
#     pre4,_ = gpr_fitting.predict_determined_input(x)
#     pre4 = pre4.reshape(-1)
#     # pre4 = pre4 - y4
#     # print("---training pre4 success---")

#     # print("---start training pre5---")
#     gpr_fitting = sgpr(X, y5)
#     gpr_fitting.train()
#     pre5,_ = gpr_fitting.predict_determined_input(x)
#     pre5 = pre5.reshape(-1)
#     # pre5 = pre5 - y5
#     # print("---training pre5 success---")

#     # print("---start training pre6---")
#     gpr_fitting = sgpr(X, y6)
#     gpr_fitting.train()
#     pre6,_ = gpr_fitting.predict_determined_input(x)
#     pre6 = pre6.reshape(-1)
#     # pre6 = pre6 - y6
#     # print("---training pre6 success---")

#     pre = np.hstack((pre0, pre1, pre2, pre3, pre4, pre5, pre6))
#     # # pre = np.hstack((pre0, pre1))
#     # # pre = pre.reshape((2, np.shape(trainingSet)[0]))
#     # pre = pre.reshape((Robot_DOF, np.shape(trainingSet)[0]))
#     # # pre = pre.T

#     # pre_dim = np.shape(pre)[1] #列数
#     # pre_num = np.shape(pre)[0] #行数
#     # print("the pre_num size is", pre_num)
#     # print("the pre_dim size is", pre_dim)
#     return pre

# x = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21]
# pre = prediction(x)
# print("pre is", pre)

