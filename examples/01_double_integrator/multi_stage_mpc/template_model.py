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
    _x = model.set_variable(var_type='_x', var_name='x', shape=(2,1))

    # Input struct (optimization variables):
    _u = model.set_variable(var_type='_u', var_name='u', shape=(1,1))

    # approximation error from learning
    _e = model.set_variable(var_type='_p', var_name='e', shape=(1,1))

    # uncertainties
    _d1 = model.set_variable(var_type='_p', var_name='d1', shape=(1,1))
    _d2 = model.set_variable(var_type='_p', var_name='d2', shape=(1,1))
    _d = vertcat(_d1, _d2)


    # Set expression. These can be used in the cost function, as non-linear constraints
    # or just to monitor another output.
    # objective
    Q = np.array(data['Q'],dtype='float')
    R = np.array(data['R'],dtype='float')
    model.set_expression(expr_name = 'stage_cost', expr = _x.T @ Q @ _x)
    model.set_expression(expr_name = 'terminal_cost', expr = _x.T @ Q @ _x)


    # system model
    x_next = data['A'] @ _x + data['B'] @ _u + data['B'] @ _e + data['E'] @ _d
    model.set_rhs('x', x_next)

    model.setup()

    return model
