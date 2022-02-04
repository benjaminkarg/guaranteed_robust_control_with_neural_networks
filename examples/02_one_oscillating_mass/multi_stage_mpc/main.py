import numpy as np
from casadi import *
import pdb
import matplotlib.pyplot as plt
plt.ion()

from template_model import *
from template_mpc import *
from template_simulator import *


""" User settings: """
N_sim = 30
N_step = 20


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

X_traj = []
U_traj = []
W_traj = []

counter_sim = 0
counter_approaches = 0
while (counter_sim < N_sim):

    # Counter to avoid infinite loop
    counter_approaches += 1

    # generate random initial state
    x0 = np.random.uniform(np.array([[-4],[-6]]), np.array([[4],[6]]))

    # Only run simulations if part of MRPI
    if np.all(invA @ x0 < invb):

        # Increase counter for simulated trajectories
        counter_sim += 1

        # initialize modules
        mpc.x0 = x0
        mpc.set_initial_guess()
        simulator.x0 = x0
        estimator.x0 = x0

        # evaluate trajectory
        for k in range(N_step):
            u0 = mpc.make_step(x0)
            y_next = simulator.make_step(u0)
            x0 = estimator.make_step(y_next)

        # save results
        X_traj.append(np.copy(simulator.data['_x']))
        U_traj.append(np.copy(simulator.data['_u']))
        W_traj.append(np.copy(simulator.data['_p']))

        # reset modules
        mpc.reset_history()
        simulator.reset_history()
        estimator.reset_history()

    if (counter_approaches >= N_sim * 100):
        break


# Plot trajectories
fig, ax = plt.subplots(1, 1)
for x, u, w in zip(X_traj, U_traj, W_traj):
    ax.plot(x[:,0], x[:,1], c = 'C0', lw=0.3, alpha=0.6)
