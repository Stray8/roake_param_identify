import autograd.numpy as np
import autograd.numpy.random as npr
from autograd import value_and_grad, grad
from scipy.optimize import minimize
from autograd.misc.optimizers import adam
import matplotlib.pyplot as plt
import autograd.scipy.stats.multivariate_normal as mvn
from autograd.numpy.linalg import solve

# np.random.seed(5) #生成随机数


# class sgpr:
#     def __init__(self, X, y, likelihood_noise=0.1, restart=1):
#         '''
#         :param X: q, dq, ddq
#         :param y: output for one dim
#         :param likelihood_noise:
#         :param restart:
#         '''
#         self.X = X
#         self.y = y
#         self.input_dim = np.shape(self.X)[1] # X的列数
#         self.input_num = np.shape(self.X)[0] # X的行数
#         self.q_dim = self.input_dim // 3 # 单纯q的列数是7列
#         self.param = np.empty(self.q_dim * 4 + 2) # 建立一个空的数组
#         self.likelihood_noise = likelihood_noise
#         self.restart = restart
#         self.cov_y_y = None # 初始化参数cov_y_y
#         self.beta = None # 初始化参数beta

#     def init_random_param(self): # 生成初始的训练参数
#         # np.random.normal 函数生成一个符合正态分布的随机数数组
#         # size 指定了生成数组的大小
#         # self.input_dim 可能是某个类的属性，表示输入的维度。// 是整数除法，返回的是整数部分
#         sqrt_kls_m = 0.01 * np.random.normal(size=self.input_dim // 3 * 2) + 2 # 均值为2，标准差为0.01
#         sqrt_kls_c = 0.01 * np.random.normal(size=self.input_dim // 3 * 2) + 2 # 均值为2，标准差为0.01
#         kn_m = 0.1 * np.random.normal(size=1) # 均值为1，标准差为0.1
#         kn_c = 0.1 * np.random.normal(size=1) # 均值为1，标准差为0.1
#         self.param = np.hstack((kn_m, sqrt_kls_m, kn_c, sqrt_kls_c))

#     def set_param(self, param):
#         self.param = param.copy()
#         # 使用rbf核函数，再加上对角阵（表示噪声的方差）
#         self.cov_y_y = self.rbf(self.X, self.X, self.param) + self.likelihood_noise ** 2 * np.eye(self.input_num)
#         self.beta = solve(self.cov_y_y, self.y) # 计算beta
#         self.inv_cov_y_y = solve(self.cov_y_y, np.eye(self.input_num)) # 计算cov_y_y的逆矩阵

#     def build_objective(self, param):
#         cov_y_y = self.rbf(self.X, self.X, param)
#         cov_y_y = cov_y_y + self.likelihood_noise**2 * np.eye(self.input_num)
#         out = - mvn.logpdf(self.y, np.zeros(self.input_num), cov_y_y) # mvn.logpdf 函数计算该分布下观测值的对数概率密度
#         return out # 返回计算得到的负对数似然值

#     def train(self):
#         max_logpdf = -1e20
#         # cons = con((0.001, 10))
#         for i in range(self.restart):
#             self.init_random_param()
#             # result = minimize(value_and_grad(self.build_objective), self.param, jac=True, method='L-BFGS-B', tol=1)
#             result = minimize(value_and_grad(self.build_objective), self.param, jac=True, method='L-BFGS-B', tol=0.01)
#             logpdf = -result.fun
#             param = result.x
#             if logpdf > max_logpdf:
#                 self.param = param
#                 max_logpdf = logpdf
#         # 提前计算，做预测时可用
#         self.cov_y_y = self.rbf(self.X, self.X, self.param) + self.likelihood_noise**2 * np.eye(self.input_num)
#         self.beta = solve(self.cov_y_y, self.y)
#         self.inv_cov_y_y = solve(self.cov_y_y, np.eye(self.input_num))

#     def rbf_single(self, x, x_, param):
#         '''
#         :param x: row element is (q, dq) or (q, ddq)
#         :param x_:
#         :param param:
#         :return:
#         '''
#         kn = param[0]
#         sqrt_kls = param[1:]
#         diffs = np.expand_dims(x / sqrt_kls, 1) - np.expand_dims(x_ / sqrt_kls, 0)
#         temp1 = np.tanh(0.1 * np.sqrt(np.sum(x[:, self.q_dim:] ** 2, axis=1)).reshape(-1, 1))
#         temp2 = np.tanh(0.1 * np.sqrt(np.sum(x_[:, self.q_dim:] ** 2, axis=1)).reshape(1, -1))
#         return temp1 * (kn**2 * np.exp(-0.5 * np.sum(diffs ** 2, axis=2))) * temp2

#     def rbf(self, x, x_, param):
#         '''
#         :param x: row element is (q, dq, ddq)
#         :param x_:
#         :param param:
#         :return:
#         '''
#         x_m = np.hstack((x[:, 0:self.q_dim], x[:, self.q_dim * 2:]))
#         x__m = np.hstack((x_[:, 0:self.q_dim], x_[:, self.q_dim * 2:]))
#         param_m = param[0: (1 + self.q_dim * 2)]
#         rbf_m = self.rbf_single(x_m, x__m, param_m)

#         x_c = x[:, 0:self.q_dim * 2]
#         x__c = x_[:, 0:self.q_dim * 2]
#         param_c = param[(1 + self.q_dim * 2):]
#         rbf_c = self.rbf_single(x_c, x__c, param_c)

#         return rbf_m + rbf_c

#     def predict_determined_input(self, inputs):  # 单维GP预测
#         # inputs 是矩阵
#         cov_y_f = self.rbf(self.X, inputs, self.param)
#         mean_outputs = np.dot(cov_y_f.T, self.beta.reshape((-1, 1)))
#         # var = (self.param[0]**2 - np.diag(np.dot(np.dot(cov_y_f.T, self.inv_cov_y_y), cov_y_f))).reshape(-1, 1)
#         return mean_outputs, None

#     def test(self, X):
#         print("self.X")

class person:
    def __init__(self, name, age)
        self.name = name
        self.age = age
    def foo(self)
        print("my name is {self.name}, age is {self.age}")
