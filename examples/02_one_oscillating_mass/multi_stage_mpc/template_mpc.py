#
#   This file is part of do-mpc
#
#   do-mpc: An environment for the easy, modular and efficient implementation of
#        robust nonlinear model predictive control
#
#   Copyright (c) 2014-2019 Sergio Lucia, Alexandru Tatulea-Codrean
#                        TU Dortmund. All rights reserved
#
#   do-mpc is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as
#   published by the Free Software Foundation, either version 3
#   of the License, or (at your option) any later version.
#
#   do-mpc is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with do-mpc.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import do_mpc
import scipy.io as sio


def template_mpc(model):
    """
    --------------------------------------------------------------------------
    template_mpc: tuning parameters
    --------------------------------------------------------------------------
    """
    mpc = do_mpc.controller.MPC(model)

    data = sio.loadmat('../data/system_and_problem_matrices.mat')
    data_MRCI = sio.loadmat('../data/MRCI.mat')
    invA = np.array(data_MRCI['MRCI_A'], dtype='float')
    invb = np.array(data_MRCI['MRCI_b'], dtype='float')
    setup_mpc = {
        'n_robust': 1,
        'n_horizon': 10,
        't_step': 0.5,
        'store_full_solution':True,
    }

    mpc.set_param(**setup_mpc)

    _x = model.x
    _u = model.u

    Q = np.array(data['Q'],dtype='float')
    R = np.array(data['R'],dtype='float')

    mterm = _x['x'].T @ Q @ _x['x']                                 # terminal cost
    lterm = _x['x'].T @ Q @ _x['x'] +  _u['u'].T @ R @ _u['u']   # stage cost

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(u=0.0)

    max_x = np.array([[4.0], [6.0]])

    mpc.bounds['lower','_x','x'] = -max_x
    mpc.bounds['upper','_x','x'] =  max_x

    mpc.bounds['lower','_u','u'] = -5.0
    mpc.bounds['upper','_u','u'] =  5.0

    # add MRCI as constraint
    mpc.set_nl_cons('MRCI', invA @ _x['x'], ub = invb)

    # Uncertainties
    e_var  = np.array([-0.01,  0.01])
    w1_var = np.array([-0.05, 0.05])
    w2_var = np.array([-0.05, 0.05])

    mpc.set_uncertainty_values(e = e_var, w1 = w1_var, w2 = w2_var)

    mpc.setup()

    return mpc
