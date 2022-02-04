function [] =  compute_MRCI_rungger_tabuada(load_str_system_data, saveloc, iter_max, roh)

    % Method to compute inner approximation of the maximum robust
    % control-invariant set
    % "Computing robust controlled invariant sets of linear systems", M.
    % Rungger and P. Tabuada, 2017

    %% Load data and extract dimensions
    load(load_str_system_data);
    nx = size(A,2);
    nu = size(B,2);

    %% Define sets
    % Admissible state space
    X = Polyhedron(H_x, h_x);

    % Disturbance set
    W = Polyhedron(H_w, h_w);

    % Input set
    U = Polyhedron(H_u, h_u);

    % Unit Ball infinity norm
    H_b = [eye(nx); -eye(nx)];
    h_b = ones(2 * nx, 1);
    rohB = Polyhedron(H_b, roh * h_b);

    % Set for safety
    WrohB = plus(W, rohB);

    % System matrices
    Ainv = inv(A);
    mBU = affineMap(U, -B);

    % Algorithm loop
    R_old = X.copy();
    for i = 1:iter_max

        % Compute preimage
        R_old_shrunk = minus(R_old, WrohB);

        % Initial controller
        M = plus(R_old_shrunk, mBU);
        pre_Ri = affineMap(M, Ainv);

        % Compute new iterate
        R_new = intersect(pre_Ri, X);

        % Break condition
        R_new_plus = plus(R_new, rohB);
        if R_new_plus.contains(R_old)
            disp("Algorithm converged!");
            break;
        else
            R_old = R_new.copy();
        end

    end

    % RX = Polyhedron(H_x, [100; -50; -26.5]);
    if nx == 2
        figure();
        plot(X, 'color', 'blue', 'alpha', 0.1);
        hold on;
        plot(R_new, 'color', 'red', 'alpha', 0.2);
    end
    MRCI_A = R_new.A;
    MRCI_b = R_new.b;
    V = R_new.V;
    savestr = strcat(saveloc, 'MRCI.mat');
    save(savestr, 'MRCI_A', 'MRCI_b', 'V');

end
