clear all;
close all;
addpath('./../../auxiliary_funs/');


%% Params
u_lb = [-5.0; -5.0]; % lower bound of control input
u_ub = [ 5.0;  5.0]; % upper bound of control input


%% Load the neural network
load('./../data/nn_controller.mat');
network = make_network_input_admissible(network, u_lb, u_ub);


%% Load system data
load('./../data/system_and_problem_matrices.mat');


%% Load minimum RPI
load('./../data/iterative_min_RPI.mat');
min_RPI = Polyhedron(H{end, 1}, h{end, 1});


%% compute worst-case cost for min RPI
max_cost = max_stage_cost(network, min_RPI, Q, R, A, B, E);


%% save
save('./../data/worst_case_cost_min_RPI.mat', 'max_cost');