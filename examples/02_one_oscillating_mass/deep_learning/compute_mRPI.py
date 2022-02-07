import numpy as np
import sys
sys.path.append('./../../../auxiliary_funs')
from aux_funs import *


""" Params """
savename = '../data/data_based_mRPI.mat'
Hp = np.random.uniform(-1, 1, )


""" Load data """



""" Generate data """
approx_mRPI(X, Hp, data_split_size, hp_split_size):(mpc, u_lb, u_ub, simulator, estimator, n_steps, savename)
