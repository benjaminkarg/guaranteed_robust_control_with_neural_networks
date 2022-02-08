import numpy as np
from casadi import *
import pdb
import matplotlib.pyplot as plt
plt.ion()

from template_model import *
from template_mpc import *
from template_simulator import *


""" User settings: """
N_sim = 50
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

X_traj = []
U_traj = []
W_traj = []

x_low = np.tile(np.array([[-4],[-2]]), (3, 1))
x_up  = np.tile(np.array([[ 4],[ 2]]), (3, 1))

counter_sim = 0
counter_approaches = 0
while (counter_sim < N_sim):

    # Counter to avoid infinite loop
    counter_approaches += 1

    # generate random initial state
    x0 = np.random.uniform(x_low, x_up)

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

        # break if infeasibilites occurr
        if not mpc.solver_stats['success']:
            break

    # save results only when no infeasibilites encountered
    if not not mpc.solver_stats['success']:
        # Increase counter for successfully simulated trajectories
        counter_sim += 1
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
fig, ax = plt.subplots(1, 3, figsize=(18,6))
for x, u, w in zip(X_traj, U_traj, W_traj):
    ax[0].plot(x[:,0], x[:,1], c = 'C0', lw=0.3, alpha=0.6)
    ax[1].plot(x[:,2], x[:,3], c = 'C0', lw=0.3, alpha=0.6)
    ax[2].plot(x[:,4], x[:,5], c = 'C0', lw=0.3, alpha=0.6)
