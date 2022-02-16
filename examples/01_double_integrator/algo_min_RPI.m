clear all;
close all;
addpath('../../auxiliary_funs/');

%% Params
exacter = false;
u_ub = [ 1.0];
u_lb = [-1.0];
k_max = 100;


%% Load the neural network
load('./data/nn_controller.mat');
network = make_network_input_admissible(network, u_lb, u_ub);


%% Load system
load('./data/system_and_problem_matrices.mat');


%% Load control invariant set
% load('./data/verification_MRCI_candidate.mat');
load('./data/MRCI.mat');
% A_inv = H{end};
% b_inv = h{end};
% X_s = Polyhedron(A_inv, b_inv);
X_s = Polyhedron(MRCI_A, MRCI_b);

% admissible state space
X = Polyhedron(H_x, h_x);

% disturbance set
D = Polyhedron(H_d, h_d);


%% Generate hyperplane directions
n_comb = 5;
nx = size(H_x, 2);
Hp_plus = combinator(n_comb, nx, 'p', 'r');        
Hp_plus = (Hp_plus - 1) / (n_comb - 1) * 2 - 1;   % Scale from -1 to 1
Hp_plus = Hp_plus(any(Hp_plus, 2), :);                 % remove all  zeros row

if exacter
    Hp = [MRCI_A; Hp_plus];
else
    Hp = MRCI_A;
end


%% Use YALMIP
tic;
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
comp_time = toc;


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
if exacter
    save('data/iterative_min_RPI_exacter.mat', 'H', 'h', 'comp_time');
else
    save('data/iterative_min_RPI.mat', 'H', 'h', 'comp_time');
end