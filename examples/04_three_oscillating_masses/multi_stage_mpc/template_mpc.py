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
import itertools
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

    max_x = np.array([[4.0], [2.0]])

    mpc.bounds['lower','_x','x'] = -np.tile(max_x, (3, 1))
    mpc.bounds['upper','_x','x'] =  np.tile(max_x, (3, 1))

    max_u = np.array([[5.0]])
    mpc.bounds['lower','_u','u'] = -np.tile(max_u, _u.shape)
    mpc.bounds['upper','_u','u'] =  np.tile(max_u, _u.shape)

    # H_x_bar = np.array(invA, dtype='float')
    # h_x_bar = np.array(invb, dtype='float')
    # mpc.set_nl_cons('X_bar', invA @ _x['x'], ub = invb)

    # Uncertainties
    e_low = -np.ones(_u.shape) * 0.01
    w_low = -np.ones(_x.shape) * 0.05
    e_up  =  np.ones(_u.shape) * 0.01
    w_up  =  np.ones(_x.shape) * 0.05
    p_low =  np.vstack([e_low, w_low])
    p_up  =  np.vstack([e_up, w_up])
    p_template = mpc.get_p_template(2)
    p_template['_p', 0] = p_low
    p_template['_p', 1] = p_up
    def p_fun(t_now):
        return p_template
    mpc.set_p_fun(p_fun)

    mpc.setup()

    return mpc
