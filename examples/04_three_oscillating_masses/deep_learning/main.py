import numpy as np
from casadi import *
import pdb
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from matplotlib import rcParams
rcParams['text.usetex'] = True
rcParams['text.latex.preamble'] = r'\usepackage{amsmath}'
rcParams['legend.fontsize'] = 'xx-large'
rcParams['axes.grid'] = True
rcParams['lines.linewidth'] = 0.5
rcParams['axes.labelsize'] = 'xx-large'
rcParams['xtick.labelsize'] = 'xx-large'
rcParams['ytick.labelsize'] = 'xx-large'
plt.ion()

import pypoman as pm
import scipy.io as asio

import sys
sys.path.append('../multi_stage_mpc')
import tensorflow as tf

from template_model import *
from template_simulator import *


""" User settings: """
N_sim = 100
N_step = 20
u_lb = np.array([[-5.0], [-5.0], [-5.0]])
u_ub = np.array([[ 5.0], [ 5.0], [ 5.0]])
x_lb = np.tile(np.array([[-1.5], [-1.5]]), (3, 1))
x_ub = np.tile(np.array([[ 1.5], [ 1.5]]), (3, 1))


"""
Get configured do-mpc modules:
"""
model = template_model()
mpc = tf.keras.models.load_model('../data/nn_controller.h5')
simulator = template_simulator(model)
estimator = do_mpc.estimator.StateFeedback(model)


""" Robust sets """
data = sio.loadmat('../data/system_and_problem_matrices.mat')

X_traj = []
U_traj = []
W_traj = []

counter_sim = 0
counter_approaches = 0
while (counter_sim < N_sim):

    # Counter to avoid infinite loop
    counter_approaches += 1

    # generate random initial state
    x0 = np.random.uniform(x_lb, x_ub)

    # Only run simulations if part of MRPI
    if True: #np.all(invA @ x0 < invb):

        # Increase counter for simulated trajectories
        counter_sim += 1

        # initialize modules
        simulator.x0 = x0
        estimator.x0 = x0

        # evaluate trajectory
        for k in range(N_step):
            u0 = u0 = mpc.predict(x0.T)
            u0_sat = np.minimum(np.maximum(u0.T, u_lb), u_ub)
            y_next = simulator.make_step(u0_sat)
            x0 = estimator.make_step(y_next)

        # save results
        X_traj.append(np.copy(simulator.data['_x']))
        U_traj.append(np.copy(simulator.data['_u']))
        W_traj.append(np.copy(simulator.data['_p']))

        # reset modules
        simulator.reset_history()
        estimator.reset_history()

    if (counter_approaches >= N_sim * 100):
        break


# Plot trajectories
fig, ax = plt.subplots(1, 3, figsize=(8,4))
for x, u, w in zip(X_traj, U_traj, W_traj):
    ax[0].plot(x[:,0], x[:,1], c = 'C0', lw=0.3, alpha=0.6)
    ax[1].plot(x[:,2], x[:,3], c = 'C0', lw=0.3, alpha=0.6)
    ax[2].plot(x[:,4], x[:,5], c = 'C0', lw=0.3, alpha=0.6)
