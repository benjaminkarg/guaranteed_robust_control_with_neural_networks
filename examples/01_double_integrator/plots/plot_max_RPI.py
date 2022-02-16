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

""" Load initial set """
data_algo = sio.loadmat('./../data/iterative_max_RPI.mat')
H = [el[0] for el in data_algo['H']]
h = [el[0] for el in data_algo['h']]


for i, (H_0, h_0, H_1, h_1) in enumerate(zip(H[:-2], h[:-2], H[1:-1], h[1:-1])):

    fig, ax = plt.subplots(figsize=(5.5, 4.5))

    # load verification data
    data_verification = sio.loadmat(f'./../data/algo_max_RPI_iter_{i+1}.mat')
    H_v = [el[0] for el in data_verification['H'][1:]]
    h_v = [el[0] for el in data_verification['h'][1:]]

    # Plot the intermediate sets
    for (H_el, h_el) in zip(H_v, h_v):
        V = pm.compute_polygon_hull(H_el, np.squeeze(h_el))
        P = plt.Polygon(V, facecolor = (0.0, 0.0, 0.0, 0.01), edgecolor = (0.0, 0.0, 0.0, 0.2), linewidth = 1.0)
        ax.add_patch(P)

    # Plot the admissible state space
    H_x = np.vstack([np.eye(2), -np.eye(2)])
    h_x = np.ones(4, ) * 5.0
    V = pm.compute_polygon_hull(H_x, h_x)
    P = plt.Polygon(V, edgecolor = 'k', fill = False, linewidth = 3.0, label = r'$\mathcal{X}$')
    ax.add_patch(P)

    # plot the initial set
    V = pm.compute_polygon_hull(H_0, np.squeeze(h_0))
    P = plt.Polygon(V, fill = False, edgecolor = 'C0', linewidth = 3.0, label = r'$\mathcal{X}_{\text{s}}$')
    ax.add_patch(P)

    # plot the preimage
    V = pm.compute_polygon_hull(H_1, np.squeeze(h_1))
    P = plt.Polygon(V, fill = False, edgecolor = 'C1', linewidth = 3.0, label = r'$\mathcal{X}_{\text{pre}}^*$')
    ax.add_patch(P)

    if i == 2:

        # plot the analytical MRCI
        """ Load algo data """
        data_MRCI = sio.loadmat('./../data/verification_MRCI.mat')
        MRCI_A = data_MRCI['H'][0,0]
        MRCI_b = data_MRCI['h'][0,0]
        V = pm.compute_polygon_hull(MRCI_A, np.squeeze(MRCI_b))
        P = plt.Polygon(V, fill = False, edgecolor = 'C2', linewidth = 3.0, linestyle = ':', label = r'$\mathcal{C}_{\text{inv}}^*$')
        ax.add_patch(P)

    # Post-processing
    ax.legend(framealpha = 0.97)
    ax.set_xlabel(r'$s~[\text{m}]$')
    if i == 0:
        ax.set_ylabel(r'$v~[\frac{\text{m}}{\text{s}}]$')
    ax.relim()
    ax.autoscale_view()
    fig.tight_layout()
    plt.savefig(f'algo_max_RPI_iter_{i + 1}.pdf', format = 'pdf')


fig, ax = plt.subplots(figsize=(6, 4.5))

# Plot the admissible state space
H_x = np.vstack([np.eye(2), -np.eye(2)])
h_x = np.ones(4, ) * 5.0
V = pm.compute_polygon_hull(H_x, h_x)
P = plt.Polygon(V, edgecolor = 'k', fill = False, linewidth = 3.0, label = r'$\mathcal{X}$')
ax.add_patch(P)

# plot the analytical MRCI
""" Load algo data """
data_MRCI = sio.loadmat('./../data/verification_MRCI.mat')
MRCI_A = data_MRCI['H'][0,0]
MRCI_b = data_MRCI['h'][0,0]
V = pm.compute_polygon_hull(MRCI_A, np.squeeze(MRCI_b))
P = plt.Polygon(V, fill = False, edgecolor = 'C0', linewidth = 3.0, linestyle = ':', label = r'$\mathcal{C}_{\text{inv}}^*$')
ax.add_patch(P)

# plot the resulting set
V = pm.compute_polygon_hull(H[-1], np.squeeze(h[-1]))
P = plt.Polygon(V, fill = False, edgecolor = 'C1', linewidth = 3.0, label = r'$\tilde{\mathcal{X}}_{\text{MRPI}}^*$')
ax.add_patch(P)

# Post-processing
ax.legend(framealpha = 0.97)
ax.set_xlabel(r'$s~[\text{m}]$')
ax.set_ylabel(r'$v~[\frac{\text{m}}{\text{s}}]$')
ax.relim()
ax.autoscale_view()
fig.tight_layout()
plt.savefig(f'algo_max_RPI_iter_{i + 2}.pdf', format = 'pdf')

# """ Plot the sets """
# fig, ax = plt.subplots(figsize=(8, 6))
#
# # Plot the admissible state space
# H_x = np.vstack([np.eye(2), -np.eye(2)])
# h_x = np.ones(4, ) * 5.0
# V = pm.compute_polygon_hull(H_x, h_x)
# P = plt.Polygon(V, edgecolor = 'k', fill = False, linewidth = 3.0, label = r'$\mathcal{X}$')
# ax.add_patch(P)
#
#
# # Plot initial set
# V = pm.compute_polygon_hull(data_init['MRCI_A'], np.squeeze(data_init['MRCI_b']))
# P = plt.Polygon(V, fill = False, edgecolor = 'C0', linewidth = 4.0, label = r'$\mathcal{X}_{\text{s}}$')
# ax.add_patch(P)
#
#
# # Plot the intermediate sets
# for (H_el, h_el) in zip(H, h):
#
#     V = pm.compute_polygon_hull(H_el, np.squeeze(h_el))
#     P = plt.Polygon(V, fill = False, edgecolor = (0.0, 0.0, 0.0, 0.4), linewidth = 1.2)
#     ax.add_patch(P)
#
#
# # plot approximate mRPI
# V = pm.compute_polygon_hull(H[-1], np.squeeze(h[-1]))
# P = plt.Polygon(V, fill = False, edgecolor = 'C1', linewidth = 3.0, label = r'$\tilde{\mathcal{X}}_{\text{mRPI}}$')
# ax.add_patch(P)
