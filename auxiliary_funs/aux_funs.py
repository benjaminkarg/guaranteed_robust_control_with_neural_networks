import numpy as np
import scipy.io as sio
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


def approx_mRPI(X, Hp, data_split_size, hp_split_size, savename):
    """
        X contains all data points: n_points x nx
        Hp are the directions of the hyperplanes: n_hp x nx
        data_split_size: number of data points to consider at one time
        hp_split_size: number of hyperplanes to fit at one time
    """

    n_data_splits = np.ceil(x_raw.shape[0]/data_split_size)
    X_split = [x_raw[i*data_split_size:(i+1)*data_split_size,:] for i in range(int(n_data_splits))]

    n_hp_splits = np.ceil(Hp.shape[0]/hp_split_size)
    HP_split = [A_raw[i*hp_split_size:(i+1)*hp_split_size,:] for i in range(int(n_hp_splits))]

    """ Compute polyhedron """
    B = []
    counter = 1
    for x in X_split:
        b_mean = []
        for A in HP_split:
            b = SX.sym("b", A.shape[0], 1)
            print(f'\n\n\nSolvin problem {counter} of {int(n_data_splits*n_hp_splits)}\n\n\n')
            g = [(A@x[i,:].reshape(-1,1) - b) for i in range(x.shape[0])]
            g.append(-b) # b > 0
            nlp = {'x':b, 'f':np.ones((1,A.shape[0]))@b, 'g':vertcat(*g)}
            S = nlpsol('S', 'ipopt', nlp)
            r = S(lbg=-inf, ubg = 0)
            b_res = np.reshape(r['x'], (-1, 1))

            counter += 1
            b_mean.append(b_res)

        B.append(np.vstack(b_mean))

    """ Get final bounds """
    b_final = np.reshape(np.max(np.hstack(B), axis=1), (-1, 1))


    """ Save results """
    exp_dic = {'mRPI_A': Hp, 'mRPI_b': np.reshape(b_final, (-1, 1))}
    sio.savemat(savename, exp_dic)
