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
import sys
sys.path.append('../data/')


def template_model(symvar_type='SX'):
    """
    --------------------------------------------------------------------------
    template_model: Variables / RHS / AUX
    --------------------------------------------------------------------------
    """
    # load data
    data = sio.loadmat('../data/system_and_problem_matrices.mat')

    model_type = 'discrete' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type, symvar_type)

    # Simple oscillating masses example with two masses and two inputs.
    # States are the position and velocitiy of the two masses.

    # States struct (optimization variables):
    nx = data['A'].shape[1]
    _x = model.set_variable(var_type='_x', var_name='x', shape=(nx,1))

    # Input struct (optimization variables):
    nu = data['B'].shape[1]
    _u = model.set_variable(var_type='_u', var_name='u', shape=(nu,1))

    # approximation error from learning
    _e = model.set_variable(var_type='_p', var_name='e', shape=(nu,1))

    # uncertainties
    nw = data['E'].shape[1]
    _w = model.set_variable(var_type='_p', var_name='w', shape=(nw, 1))


    # Set expression. These can be used in the cost function, as non-linear constraints
    # or just to monitor another output.
    # objective
    Q = np.array(data['Q'], dtype='float')
    R = np.array(data['R'], dtype='float')
    model.set_expression(expr_name = 'stage_cost', expr = _x.T @ Q @ _x)
    model.set_expression(expr_name = 'terminal_cost', expr = _x.T @ Q @ _x)


    # system model
    A = np.array(data['A'], dtype='float')
    B = np.array(data['B'], dtype='float')
    E = np.array(data['E'], dtype='float')
    x_next = A @ _x + B @ _u + B @ _e + E @ _w
    model.set_rhs('x', x_next)

    model.setup()

    return model
