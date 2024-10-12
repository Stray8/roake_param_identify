import numpy as np
import cvxopt
from cvxopt import matrix, solvers


def construct_A(_A, H):
    '''
    :param _A: dim_A * dim_A, \overline A
    :param H: 1, Horizon
    :return:  (H * dim_A) * dim_A,\mathcal{A}
    '''
    A = _A
    for i in range(H):
        if i != 0:
            A = np.vstack((A, np.linalg.matrix_power(_A, i+1)))
    return A

def construct_B(_A, _B, H):
    '''
    :param _A: dim_A * dim_A, \overline A
    :param _B: dim_A * 1, \overline B
    :param H: 1, Horizon
    :return:  (H * dim_A) * H,\mathcal{B}
    '''
    dim_A = np.shape(_A)[0]
    B = np.zeros((H * dim_A, H))
    for i in range(H):
        for j in range(H):
            if j > i:
                B[i * dim_A: (i + 1) * dim_A, j] = np.zeros(dim_A)
            else:
                B[i * dim_A: (i + 1) * dim_A, j] = np.linalg.matrix_power(_A, i - j).dot(_B).reshape(-1)
    return B

def construct_C(_A, _C, H):
    '''
    :param _A: dim_A * dim_A, \overline A
    :param _C: dim_A * dim_A, \overline B
    :param H: 1, Horizon
    :return:  (H * dim_A) * (H * dim_A),\mathcal{B}
    '''
    dim_A = np.shape(_A)[0]
    C = np.zeros((H * dim_A, H * dim_A))
    for i in range(H):
        for j in range(H):
            if j > i:
                C[i * dim_A: (i + 1) * dim_A, j * dim_A: (j + 1) * dim_A] = np.zeros((dim_A, dim_A))
            else:
                C[i * dim_A: (i + 1) * dim_A, j * dim_A: (j + 1) * dim_A] = np.linalg.matrix_power(_A, i - j).dot(_C)
    return C

def construct_sr(sr_matrix):
    '''
    :param sr_matrix: (H * dim_A) * dim_A
    :return: (H * dim_A * dim_A) * 1
    '''
    return sr_matrix.reshape(-1, 1)

def construct_cons_vector(cons, H):
    '''
    :param x_m: 1, minimal x
    :param x_M: 1, maximal x
    :param dx_m: 1, minimal dx
    :param dx_M: 1, maximal dx
    :param H: 1, Horizon
    :return: (4 * H ) * 1
    '''
    x_m, x_M, dx_m, dx_M = cons
    x_m_cons = -np.ones(H) * x_m
    x_M_cons = np.ones(H) * x_M
    dx_m_cons = -np.ones(H) * dx_m
    dx_M_cons = np.ones(H) * dx_M
    cons_vector = np.hstack((x_m_cons, x_M_cons, dx_m_cons, dx_M_cons))
    return cons_vector

def Psi(beta, H):
    '''
    :param beta: dim_A * 1
    :param H: 1
    :return: H * (H * dim_A)
    '''
    dim_A = np.shape(beta)[0]
    Psi = np.zeros((H, H * dim_A))
    for i in range(H):
        for j in range(H):
            if i == j:
                Psi[i, j * dim_A:(j+1) * dim_A] = beta.reshape(-1)
    return Psi

def Phi(gamma, H):
    '''
    :param beta: dim_A * 1
    :param H: 1
    :return: H * (H * dim_A)
    '''
    dim_A = np.shape(gamma)[0]
    Phi = np.zeros((H, H * dim_A))
    for i in range(H):
        for j in range(H):
            if i == j:
                Phi[i, j * dim_A:(j+1) * dim_A] = gamma.reshape(-1)
    return Phi



class Qp2pvc:
    def __init__(self, H, cons, W):
        self.H = H
        self.cons_vector = construct_cons_vector(cons, H)
        self.W = W

    def computing_f_seq(self, _A, _B, _C, s, sr_matrix):
        H = self.H


        A = construct_A(_A, H)
        B = construct_B(_A, _B, H)
        C = construct_C(_A, _C, H)
        sr = construct_sr(sr_matrix)
        beta = np.array([1, 0, 0]).reshape(3, 1)
        gamma = np.array([0, 1, 0]).reshape(3, 1)

        psi = Psi(beta, H)
        phi = Phi(gamma, H)
        cons_vector = self.cons_vector

        P = self.W
        q = np.zeros(H)
        temp = np.vstack((-psi, psi, -phi, phi))
        G = temp.dot(B)
        h = cons_vector - temp.dot(A.dot(s.reshape(-1, 1)) - C.dot(sr)).reshape(-1)

        print('A shape is ', np.shape(A))
        print('B shape is ', np.shape(B))
        print('C shape is ', np.shape(C))
        print('G shape is ', np.shape(G))
        print('h shape is ', np.shape(h))

        P = matrix(P)
        q = matrix(q)
        G = matrix(G)
        h = matrix(h.reshape(-1))

        sol = solvers.qp(P=P, q=q, G=G, h=h)
        return np.array(sol['x'])



