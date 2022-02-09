import numpy as np
import scipy.io as sio
from casadi import *
import multiprocessing as mp
import pdb


def gen_data_mRPI(nn_controller, u_lb, u_ub, simulator, estimator, n_steps, savename):
    """ Simulate a system with a given controller for n_steps steps """

    """ Extract size and set initial values """
    x0 = np.zeros(simulator.model.x.shape)
    simulator.x0 = x0
    estimator.x0 = x0


    """ Run closed-loop simulation """
    for _ in range(n_steps):
        u0 = nn_controller.predict(x0.T)
        u0_sat = np.minimum(np.maximum(u0.T, u_lb), u_ub)
        y_next = simulator.make_step(u0.T)
        x0 = estimator.make_step(y_next)


    """ Save data """
    exp_dic = {'X': simulator.data['_x']}
    sio.savemat(savename, exp_dic)



def gen_data_closed_loop(nn_controller, u_lb, u_ub, x_lb, x_ub, H_x, h_x, simulator, estimator, n_sim, n_steps, savename):
    """ Simulate n_sim closed-loop trajectories of length n_steps """


    X = []
    U = []
    cost = []

    counter_sim = 0
    while (counter_sim < n_sim):

        # generate random initial state and set initial values
        x0 = np.random.uniform(x_lb, x_ub)
        simulator.x0 = x0
        estimator.x0 = x0


        """ Run closed-loop simulation """
        for _ in range(n_steps):
            u0 = nn_controller.predict(x0.T)
            u0_sat = np.minimum(np.maximum(u0.T, u_lb), u_ub)
            y_next = simulator.make_step(u0.T)
            x0 = estimator.make_step(y_next)

        # check if trajectory admissible
        admissible = [np.all((H_x @ np.reshape(x, (-1, 1))) <= h_x) for x in simulator.data['_x']]
        if np.all(admissible):

            # increase counter
            counter_sim += 1
            print(f'Trajectory {counter_sim}  of {n_sim}')

            # save results
            X.append(np.copy(simulator.data['_x']))
            U.append(np.copy(simulator.data['_u']))
            cost.append(np.copy(simulator.data['_aux', 'stage_cost']))

        # reset modules
        simulator.reset_history()
        estimator.reset_history()


    """ Save data """
    exp_dic = {'X': X, 'U': U, 'cost': cost}
    sio.savemat(savename, exp_dic)
