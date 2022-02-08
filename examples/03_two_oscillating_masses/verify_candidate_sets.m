clear all;
close all;
addpath('../../auxiliary_funs/');


%% Params
u_lb = [-5.0; -5.0]; % lower bound of control input
u_ub = [ 5.0;  5.0]; % upper bound of control input
r_max = 10;  % maximum number of iterations
check_potential_MRPI = false; % either check a potential MRPI derived via rungger-tabuada or use random set


%% Load the neural network
load('./data/nn_controller.mat');
network = make_network_input_admissible(network, u_lb, u_ub);


%% Load system
load('./data/system_and_problem_matrices.mat');
nx = size(A, 2);
nu = size(B, 2);
nw = size(E, 2);


%% Load and create sets for verification

% Create candidate set for r-step invariance
% in this case the MRCI via rungger-tabuada
if check_potential_MRPI
    load('./data/MRCI.mat');
    X_s = Polyhedron(MRCI_A, MRCI_b);
else
    C_A = [eye(nx); -eye(nx)];
    C_b = ones(2 * nx, 1) * 1.5;
    X_s = Polyhedron(C_A, C_b);
end

% hyperplanes to be considered for over-approximation of the one-step
% reachable sets, this case the same hyperplanes as for the MRCI
if check_potential_MRPI
    Hp = MRCI_A;
else
    rng(123456);
    Hp = rand(2000, nx) * 2 - 1;
end

% disturbance set
D = Polyhedron(H_d, h_d);

% admissible state space
X = Polyhedron(H_x, h_x);


%% check if r-step admissible set exists
tic;
[r, sets, success] = r_step_invariance(network, Hp, X, X_s, D, r_max, A, B, E);
comp_time = toc;


%% Save results
if success
    H = {};
    h = {};
    for i = 1:length(sets)
        H{i} = sets(i).A;
        h{i} = sets(i).b;
    end
    save('./data/verification_MRCI.mat', 'H', 'h', 'comp_time');
end
