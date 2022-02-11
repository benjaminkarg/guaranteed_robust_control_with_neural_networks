import numpy as np
import sys
sys.path.append('./../../../auxiliary_funs')
from aux_funs import *
import tensorflow as tf

sys.path.append('../multi_stage_mpc')
from template_model import *
from template_simulator import *
import scipy.io as sio


""" Params """
n_sim = 100
n_steps = 500
u_lb = np.array([[-1.0]])
u_ub = np.array([[ 1.0]])
x_lb = np.array([[-5.0], [-5.0]])
x_ub = np.array([[ 5.0], [ 5.0]])
savename = '../data/closed_loop_sim.mat'


""" Load system data """
data = sio.loadmat('./../data/system_and_problem_matrices.mat')
H_x = np.array(data['H_x'], dtype = 'float')
h_x = np.array(data['h_x'], dtype = 'float')


""" Load r-step invariant set """
data_MRCI = sio.loadmat('./../data/MRCI.mat')
nx = H_x.shape[1]
A_inv = np.array(data_MRCI['MRCI_A'], dtype = 'float')
b_inv = np.array(data_MRCI['MRCI_b'], dtype = 'float')


""" Get configured do-mpc modules """
model = template_model()
mpc = tf.keras.models.load_model('../data/nn_controller.h5')
simulator = template_simulator(model)
estimator = do_mpc.estimator.StateFeedback(model)


""" Generate data """
gen_data_closed_loop(mpc, u_lb, u_ub, A_inv, b_inv, x_lb, x_ub, H_x, h_x, simulator, estimator, n_sim, n_steps, savename)
