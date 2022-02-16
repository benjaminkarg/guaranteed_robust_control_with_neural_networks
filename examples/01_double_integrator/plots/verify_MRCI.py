import numpy as np
import scipy.io as sio
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
import pdb
import pypoman as pm



""" Load algo data """
data_algo = sio.loadmat('./../data/verification_MRCI.mat')
H = [data_algo['H'][0,i] for i in range(data_algo['H'].shape[1])]
h = [data_algo['h'][0,i] for i in range(data_algo['h'].shape[1])]


""" Plot the sets """
fig, ax = plt.subplots(figsize=(8, 6))

# Plot the admissible state space
H_x = np.vstack([np.eye(2), -np.eye(2)])
h_x = np.ones(4, ) * 5.0
V = pm.compute_polygon_hull(H_x, h_x)
P = plt.Polygon(V, edgecolor = 'k', fill = False, linewidth = 3.0, label = r'$\mathcal{X}$')
ax.add_patch(P)


# Plot the candidate set
V = pm.compute_polygon_hull(H[0], np.squeeze(h[0]))
P = plt.Polygon(V, edgecolor = 'C0', fill = False, linewidth = 4.0, label = r'$\mathcal{C}_{\text{inv}}$', alpha = 1.0)
ax.add_patch(P)

# Plot 1-step reachable set
V = pm.compute_polygon_hull(H[1], np.squeeze(h[1]) * 1.0)
P = plt.Polygon(V, edgecolor = 'C1', fill = False, linewidth = 4.0, label = r'$\mathcal{X}_1^*$', alpha = 1.0, linestyle = '-.')
ax.add_patch(P)


# Post-processing
ax.legend(loc = 'upper right', framealpha = 0.97)
ax.set_xlabel(r'$s~[\text{m}]$')
ax.set_ylabel(r'$v~[\frac{\text{m}}{\text{s}}]$')
ax.relim()
ax.autoscale_view()
fig.tight_layout()
plt.savefig("algo_verification_MRCI.pdf", format = 'pdf')
