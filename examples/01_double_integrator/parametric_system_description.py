import numpy as np
from itertools import product
import scipy.io as sio
import pdb

""" Auxiliary functions """
def build_system_matrices(p):

    # p: realization of uncertainties, 4 x 1

    A = np.array([[1, p[0,0]],
                  [0, p[1,0]]])

    B = np.array([[p[2, 0]], [p[3, 0]]])

    return A, B

""" Define parameter values (upper and lower bound) """
p0 = [0.9, 1.0]
p1 = [0.9, 1.0]
p2 = [0.4, 0.6]
p3 = [0.9, 1.0]


""" Obtain system descriptions """
A_list = []
B_list = []
param_combs = list(product(p0, p1, p2, p3))
for pc in param_combs:
    p_vec = np.reshape(pc, (-1, 1))
    A, B = build_system_matrices(p_vec)
    A_list.append(A)
    B_list.append(B)


""" Admissible state space """
H_x = np.vstack([np.eye(2), -np.eye(2)])
h_x = np.ones((4, 1)) * 5.0


""" Admissible control input space """
H_u = np.array([[1], [-1]])
h_u = np.ones((2, 1))


""" Approximation error """
H_delta = np.array([[1], [-1]])
h_delta = np.array([[0.01], [0.01]])


""" Save system descriptions """
exp_dic = {'H_x': H_x, 'h_x': h_x, 'H_u': H_u, 'h_u': h_u, 'H_delta': H_delta, 'h_delta': h_delta, 'A_list': A_list, 'B_list': B_list}
sio.savemat('parametric_system_and_problem_matrices.mat', exp_dic)
