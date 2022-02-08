clear all;
close all;
addpath('../../auxiliary_funs/');


%% Params
u_lb = -5.0; % lower bound of control input
u_ub =  5.0; % upper bound of control input
r_max = 10;  % maximum number of iterations

check = 'candidate'; %either 'analytical' to check MRPI derived via rungge-tabuada, 'data_based' to check based on learning data or 'candidate' 


%% Load the neural network
load('./data/nn_controller.mat');
network = make_network_input_admissible(network, u_lb, u_ub);


%% Load system
load('./data/system_and_problem_matrices.mat');


%% Load and create sets for verification

% Create candidate set for r-step invariance
% in this case the MRCI via rungger-tabuada
if strcmp(check, 'analytical')
    load('./data/MRCI.mat');
    X_s = Polyhedron(MRCI_A, MRCI_b);
%     X_s = Polyhedron(RPI_A, RPI_b);
elseif strcmp(check, 'data_based')
    load('./data/approx_max_RPI_sim_based.mat');
    X_s = Polyhedron(RPI_A, RPI_b);
elseif strcmp(check, 'candidate')
    C_A = [eye(2); -eye(2);];
    C_b = [1.5; 1.5; 1.5; 1.5];
    X_s = Polyhedron(C_A, C_b);
end

% hyperplanes to be considered for over-approximation of the one-step
% reachable sets, this case the same hyperplanes as for the MRCI
if strcmp(check, 'analytical')
    Hp = MRCI_A;
else
    rng(1234);
    Hp = rand(100, 2) * 2 - 1;
end

% disturbance set
D = Polyhedron(H_d, h_d);

% admissible state space
X = Polyhedron(H_x, h_x);


%% check if r-step admissible set exists
tic;
[r, sets, success] = r_step_invariance(network, Hp, X, X_s, D, r_max, A, B, E);
comp_time = toc;
%% Plot result
if true
    figure();
    % Plot admissible state space
    plot(X, 'color', 'red', 'alpha', 0.1);
    hold on;
    % Plot sets of the iteration (starting with the set to be verified)
    for i = 1:length(sets)
        plot(sets(i,1), 'color', 'blue', 'alpha', 0.1);
    end
end


%% Save results
if success
    H = {};
    h = {};
    for i = 1:length(sets)
        H{i} = sets(i).A;
        h{i} = sets(i).b;
    end
    if strcmp(check, 'analytical')
        save('./data/verification_MRCI_analytical.mat', 'H', 'h', 'comp_time');
    elseif strcmp(check, 'data_based')
        save('./data/verification_MRCI_data_based.mat', 'H', 'h', 'comp_time');
    elseif strcmp(check, 'candidate')
        save('./data/verification_MRCI_candidate.mat', 'H', 'h', 'comp_time');
    end
        
end
