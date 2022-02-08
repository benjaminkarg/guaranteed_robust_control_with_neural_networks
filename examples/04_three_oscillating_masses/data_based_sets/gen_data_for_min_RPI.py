import numpy as np
import sys
sys.path.append('./../../../auxiliary_funs')
from aux_funs import *
import tensorflow as tf

sys.path.append('../multi_stage_mpc')
from template_model import *
from template_simulator import *


""" Params """
u_lb = np.array([[-5.0], [-5.0], [-5.0]])
u_ub = np.array([[ 5.0], [ 5.0], [ 5.0]])
n_steps = 10000
savename = '../data/min_RPI_sim_data.mat'


""" Get configured do-mpc modules """
model = template_model()
mpc = tf.keras.models.load_model('../data/nn_controller.h5')
simulator = template_simulator(model)
estimator = do_mpc.estimator.StateFeedback(model)


""" Generate data """
gen_data_mRPI(mpc, u_lb, u_ub, simulator, estimator, n_steps, savename)
