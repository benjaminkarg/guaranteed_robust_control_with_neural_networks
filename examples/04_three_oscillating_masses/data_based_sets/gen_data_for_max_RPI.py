import numpy as np
import sys
sys.path.append('./../../../auxiliary_funs')
from aux_funs import *
import tensorflow as tf
import scipy.io as sio

sys.path.append('./../multi_stage_mpc')
from template_model import *
from template_simulator import *
import pdb


""" Params """
n_sim = 2000
n_steps = 5
u_lb = np.array([[-5.0], [-5.0], [-5.0]])
u_ub = np.array([[ 5.0], [ 5.0], [ 5.0]])
x_lb = np.tile(np.array([[-4.0], [-2.0]]), (3, 1))
x_ub = np.tile(np.array([[ 4.0], [ 2.0]]), (3, 1))
savename = '../data/max_RPI_sim_data.mat'


""" Get configured do-mpc modules """
model = template_model()
mpc = tf.keras.models.load_model('../data/nn_controller.h5')
simulator = template_simulator(model)
estimator = do_mpc.estimator.StateFeedback(model)

""" Load system and problem data """
data = sio.loadmat('./../data/system_and_problem_matrices.mat')
H_x = np.array(data['H_x'], dtype = 'float')
h_x = np.array(data['h_x'], dtype = 'float')


""" Generate data """
gen_data_max_RPI(mpc, u_lb, u_ub, x_lb, x_ub, H_x, h_x, simulator, estimator, n_sim, n_steps, savename)
