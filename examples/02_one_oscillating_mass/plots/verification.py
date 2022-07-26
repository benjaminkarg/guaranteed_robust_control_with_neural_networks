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
rcParams['lines.linewidth'] = 3.0
rcParams['axes.labelsize'] = 'xx-large'
rcParams['xtick.labelsize'] = 'xx-large'
rcParams['ytick.labelsize'] = 'xx-large'
rcParams['pdf.fonttype'] = 42
rcParams['ps.fonttype'] = 42
plt.ion()
import pdb
import pypoman as pm

""" Parameters """
type_set = 'candidate' # either 'candidate' or 'analytical'


""" Load algo data """
# Guess
data_guess = sio.loadmat(f'./../data/verification_MRCI_{type_set}.mat')
H_guess = [data_guess['H'][0,i] for i in range(data_guess['H'].shape[1])]
h_guess = [data_guess['h'][0,i] for i in range(data_guess['h'].shape[1])]

# MRCI
# data_MRCI = sio.loadmat('./../data/verification_MRCI.mat')
# H_MRCI = [data_MRCI['H'][0,i] for i in range(data_MRCI['H'].shape[1])]
# h_MRCI = [data_MRCI['h'][0,i] for i in range(data_MRCI['h'].shape[1])]


""" Plot the sets """
fig, ax = plt.subplots(figsize=(8, 6))


# Plot the admissible state space
H_x = np.vstack([np.eye(2), -np.eye(2)])
h_x = np.array([4, 2, 4, 2])
V = pm.compute_polygon_hull(H_x, h_x)
P = plt.Polygon(V, edgecolor = 'k', fill = False, linewidth = 3.0, label = r'$\mathcal{X}$')
ax.add_patch(P)


# Plot the candidate sets

# """ MRCI """
# # MRCI
# V = pm.compute_polygon_hull(data_MRCI['H_MRCI'], np.squeeze(data_MRCI['h_MRCI']))
# P = plt.Polygon(V, edgecolor = 'C0', fill = False, linewidth = 3.0, label = r'$\mathcal{C}_{\text{inv}}$', alpha = 1.0)
# ax.add_patch(P)
#
# # MRCI
# V = pm.compute_polygon_hull(H_MRCI[-1], np.squeeze(h_MRCI[-1]) * 1.0)
# P = plt.Polygon(V, edgecolor = 'C1', fill = False, linewidth = 3.0, label = r'$\tilde{\mathcal{R}}_1(\mathcal{C}_{\text{inv}})$', alpha = 1.0)
# ax.add_patch(P)


""" Guess """
# Plot intermediate sets
for i, (Hp, hp) in enumerate(zip(H_guess[:-1], h_guess[:-1])):
    V = pm.compute_polygon_hull(Hp, np.squeeze(hp) * 1.0)
    if i == 0:
        P = plt.Polygon(V, edgecolor = 'C0', fill = False, linewidth = 3.0, alpha = 1.0, linestyle = '-', label = r'$\mathcal{X}_{\text{s}}$')
    elif i == 1:
        P = plt.Polygon(V, edgecolor = 'grey', fill = False, linewidth = 3.0, alpha = 1.0, linestyle = '-.', label = r'$\tilde{\mathcal{R}}_{1}\left(\mathcal{X}_{\text{s}}\right)$')
    elif i == 2:
        P = plt.Polygon(V, edgecolor = 'grey', fill = False, linewidth = 3.0, alpha = 1.0, linestyle = ':', label = r'$\tilde{\mathcal{R}}_{2}\left(\mathcal{X}_{\text{s}}\right)$')
    ax.add_patch(P)

# Guess
# V = pm.compute_polygon_hull(data_guess['H_s'], np.squeeze(data_guess['h_s']))
# P = plt.Polygon(V, edgecolor = 'C0', fill = False, linewidth = 3.0, label = r'$\mathcal{C}_{\text{guess}}$', alpha = 1.0, linestyle = '-.')
# ax.add_patch(P)


# Plot final sets
# Guess
V = pm.compute_polygon_hull(H_guess[-1], np.squeeze(h_guess[-1]) * 1.0)
P = plt.Polygon(V, edgecolor = 'C1', fill = False, linewidth = 3.0, label = r'$\tilde{\mathcal{R}}_2(\mathcal{X}_{\text{s}})$', alpha = 1.0, linestyle = '-')
ax.add_patch(P)


# Post-processing
ax.legend(loc = 'upper right', framealpha = 0.97, ncol = 1)
ax.set_xlabel(r'$s~[\text{m}]$')
ax.set_ylabel(r'$v~[\frac{\text{m}}{\text{s}}]$')
ax.set_xlim([-4.5, 4.5])
ax.set_ylim([-2.5, 2.5])
ax.autoscale_view()
fig.tight_layout()
plt.savefig(f"verification_MRCI_{type_set}.pdf", format = 'pdf')
