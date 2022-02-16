import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
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
import pdb


""" Load simulation data """
sim_data = sio.loadmat('./../data/closed_loop_sim.mat')
cost = sim_data['cost']


""" Compute average cost """
avg_cost = []
for c_traj in cost:
    avg_cost_i = []
    for i in range(c_traj.shape[0]):
        avg_cost_i.append( np.sum(c_traj[:i+1]) / (i + 1))
    avg_cost.append(np.vstack(avg_cost_i))


""" Plot average cost """
fig, ax = plt.subplots(1, 1, figsize = (4.5, 4.5))
# ax.set_yscale('log')
for avg_cost_traj in avg_cost:
    ax.plot(avg_cost_traj, c = 'C0', lw = 0.3)
ax.set_ylabel(r'$l_k(x_0)$')
ax.set_xlabel(r'$k$')
# ax.set_title('Avg. stage cost')



# plot cost bound
max_cost = sio.loadmat('./../data/worst_case_cost_min_RPI.mat')['max_cost']
ub = np.ones((avg_cost[0].shape[0], 1)) * max_cost;
ax.plot(ub, c = 'C1', lw = 3.0, ls = '-.', label = r'$\ell_{\infty}$')
ax.set_xlim([0, 25])
# ax.set_ylim([0, 20])
ax.legend()
fig.tight_layout()
plt.savefig("average_stage_cost_one_oscillating_mass.pdf", format = 'pdf')
