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
N_sim = 300
N_step = 20
u_lb = np.array([[-5.0]])
u_ub = np.array([[ 5.0]])


"""
Get configured do-mpc modules:
"""
model = template_model()
mpc = tf.keras.models.load_model('../data/nn_controller.h5')
simulator = template_simulator(model)
estimator = do_mpc.estimator.StateFeedback(model)


""" Robust sets """
data = sio.loadmat('../data/system_and_problem_matrices.mat')
data_MRCI = sio.loadmat('../data/MRCI.mat')
invA = np.array(data_MRCI['MRCI_A'], dtype='float')
invb = np.array(data_MRCI['MRCI_b'], dtype='float')

X_traj = []
U_traj = []
W_traj = []

counter_sim = 0
counter_approaches = 0
while (counter_sim < N_sim):

    # Counter to avoid infinite loop
    counter_approaches += 1

    # generate random initial state
    x0 = np.random.uniform(np.array([[-2],[-5]]), np.array([[2],[5]]))

    # Only run simulations if part of MRPI
    if np.all(invA @ x0 < invb):

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


# Plot MRCI and trajectories
fig, ax = plt.subplots(1, 1)
V = pm.compute_polygon_hull(invA, np.squeeze(invb))
P = plt.Polygon(V, facecolor= (0.1, 0.2, 0.5, 0.2), edgecolor = (0.0, 0.0, 0.0, 1.0), linewidth = 1.0, label='MRCI')
ax.add_patch(P)
for x, u, w in zip(X_traj, U_traj, W_traj):
    ax.plot(x[:,0], x[:,1], c = 'C0', lw=0.4, alpha=0.6)


# Check if any non-admissible values
x_data = np.vstack(X_traj)
n_viols = 0
for i in range(x_data.shape[0]):
    x_cur = x_data[i,:].reshape(-1, 1)
    if not np.all(invA @ x_cur <= invb):
        n_viols += 1
print(f'{n_viols} constraint violations occurred!')
