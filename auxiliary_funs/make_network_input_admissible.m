function [saturated_network] = make_network_input_admissible(network, u_lb, u_ub)
%In case of box constraints, ensure that input constraints are always
%satisfied by adding adding more layers
%"Stability and feasibility of neural network-based controllers via
%output-range analysis", B. Karg and S. Lucia, 2020

    % Extract weights and biases, and check number of hidden layers
    weights = network.weights;
    biases = network.biases;
    nu = max(size(u_lb));

    % store weights of final layer
    W_final = weights{1,end};
    b_final = biases{1,end};

    % Adapat previously final layer
    weights{1,end} = -W_final;
    biases{1,end} = u_ub - b_final;

    % Add layers
    weights{1, end + 1} = -eye(nu);
    biases{1, end + 1} = u_ub - u_lb;
    weights{1, end + 1} = eye(nu);
    biases{1, end + 1} = u_lb;

    % build saturated network
    saturated_network.weights = weights;
    saturated_network.biases = biases;

end

