import numpy as np
from casadi import *
import pdb
import matplotlib.pyplot as plt
plt.ion()

import sys
sys.path.append('../multi_stage_mpc')

from template_model import *
from template_mpc import *
from template_simulator import *


""" User settings: """
np.random.seed(400)
n_samples = 4000
save_results = True


"""
Get configured do-mpc modules:
"""
model = template_model()
mpc = template_mpc(model)
simulator = template_simulator(model)
estimator = do_mpc.estimator.StateFeedback(model)


""" Robust sets """
data = sio.loadmat('../data/system_and_problem_matrices.mat')
data_MRCI = sio.loadmat('../data/MRCI.mat')
invA = np.array(data_MRCI['MRCI_A'], dtype='float')
invb = np.array(data_MRCI['MRCI_b'], dtype='float')

# Bounds for sampling state space
x_min = np.array([[-4.0], [-2.0]])
x_max = np.array([[ 4.0], [ 2.0]])

# data storage
X = []
U_robust = []

c_samples = 0
while (c_samples < n_samples):

    x_pot = np.random.uniform(x_min, x_max)

    # check if x_pot belongs to MRCI
    if np.all(invA @ x_pot < invb):

        # increase sample counter
        c_samples += 1
        print(f'\n\nSample {c_samples}/{n_samples}\n\n')

        # initialize do-mpc modules
        mpc.x0 = x_pot
        mpc.set_initial_guess()
        simulator.x0 = x_pot
        estimator.x0 = x_pot

        # one iteration of mpc loop
        u0 = mpc.make_step(x_pot)
        y_next = simulator.make_step(u0)
        x0 = estimator.make_step(y_next)

        # save results
        X.append(np.reshape(x_pot, (-1,1)))
        U_robust.append(np.reshape(u0, (-1,1)))

        # reset modules
        mpc.reset_history()
        simulator.reset_history()
        estimator.reset_history()


# Save results
if save_results:
    exp_dic = {'X': np.hstack(X), 'U': np.hstack(U_robust)}
    sio.savemat('../data/data_learning.mat', exp_dic)

# plot data points
X_plot = np.hstack(X)
fig, ax = plt.subplots()
ax.scatter(X_plot[0,:],X_plot[1,:])
