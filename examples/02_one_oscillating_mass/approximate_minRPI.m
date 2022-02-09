clear all;
close all;
addpath('../../auxiliary_funs/');

%% Params
u_ub =  5.0;
u_lb = -5.0;
k_max = 30;


%% Load the neural network
load('./data/nn_controller.mat');
network = make_network_input_admissible(network, u_lb, u_ub);


%% Load system
load('./data/system_and_problem_matrices.mat');


%% Load control invariant set
load('./data/verification_MRCI_analytical.mat');
A_inv = H{2};
b_inv = h{2};
X_s = Polyhedron(A_inv, b_inv);

% admissible state space
X = Polyhedron(H_x, h_x);

% disturbance set
D = Polyhedron(H_d, h_d);


%% Generate hyperplane directions
n_comb = 5;
Hp = combinator(n_comb, 2, 'p', 'r');        
Hp = (Hp - 1) / (n_comb - 1) * 2 - 1;   % Scale from -1 to 1
Hp = Hp(any(Hp, 2), :);                 % remove all  zeros row


%% Use YALMIP
iter_sets = [];
for k = 1:k_max
    
    % Compute succesor set
    [r, sets, success] = r_step_invariance(network, Hp, X, X_s, D, 1, A, B, E);
    
    % Check if succesor set
    if success
        iter_sets = horzcat(iter_sets, sets(1));
        X_s = sets(2);
    else
        break
    end
    
end


%% Plot result
figure();
plot(X, 'color', 'red', 'alpha', 0.1);
hold on;
for i = 1:length(iter_sets)
    plot(iter_sets(i,1), 'color', 'blue', 'alpha', 0.1);
end


%% save results
H = {};
h = {};
for i = 1:length(iter_sets)
    H{i,1} = iter_sets(i,1).A;
    h{i,1} = iter_sets(i,1).b;
end
save('data/iterative_min_RPI.mat', 'H', 'h');