clear all;
close all;
addpath('../../auxiliary_funs/');


%% Params
u_lb = [-5.0; -5.0; -5.0]; % lower bound of control input
u_ub = [ 5.0;  5.0;  5.0]; % upper bound of control input
r_max = 10;  % maximum number of iterations
check = 'candidate'; % either 'data_based' set or 'candidate' set, that was chosen


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
if strcmp(check, 'data_based')
    load('./data/approx_max_RPI_sim_based.mat');
    X_s = Polyhedron(RPI_A, RPI_b * 0.9);
elseif strcmp(check, 'candidate')
    C_A = [eye(nx); -eye(nx)];
    C_b = ones(2 * nx, 1) * 1.5;
    X_s = Polyhedron(C_A, C_b);
end

% hyperplanes to be considered for over-approximation of the one-step
% reachable sets, this case the same hyperplanes as for the MRCI
if strcmp(check, 'data_based')
    Hp = RPI_A;
elseif strcmp(check, 'candidate')
%     rng(1256);
%     Hp = rand(10000, nx) * 2 - 1;
    n_comb = 3;
    Hp = combinator(n_comb, nx, 'p', 'r');
    Hp = (Hp - 1) / (n_comb - 1) * 2 - 1;   % Scale from -1 to 1
    Hp = Hp(any(Hp, 2), :);                 % remove all zeros row
    
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
    if strcmp(check, 'data_based')
        save('./data/verification_MRPI_data_based.mat', 'H', 'h', 'comp_time');
    elseif strcmp(check, 'candidate')
        save('./data/verification_MRPI_candidate.mat', 'H', 'h', 'comp_time');
    end
end
