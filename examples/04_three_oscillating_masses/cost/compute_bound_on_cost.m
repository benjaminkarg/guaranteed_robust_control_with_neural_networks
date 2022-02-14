clear all;
close all;
addpath('./../../auxiliary_funs/');


%% Params
u_lb = [-5.0; -5.0; -5.0]; % lower bound of control input
u_ub = [ 5.0;  5.0;  5.0]; % upper bound of control input


%% Load the neural network
load('./../data/nn_controller.mat');
network = make_network_input_admissible(network, u_lb, u_ub);


%% Load system data
load('./../data/system_and_problem_matrices.mat');


%% Load analytic approximation of minimum RPI
load('./../data/iterative_min_RPI.mat');
min_RPI_analytic = Polyhedron(H{end, 1}, h{end, 1});


%% load data-based approximation of minimum RPI
load('./../data/approx_min_RPI_sim_based.mat');
min_RPI_data = Polyhedron(RPI_A, RPI_b);


%% compute worst-case cost for milp - based min RPI
max_cost_milp = max_stage_cost(network, min_RPI_analytic, Q, R, A, B, E);


%% compute worst-case cost for data - based min RPI
% max_cost_data = max_stage_cost(network, min_RPI_data, Q, R, A, B, E);


%% save
% save('./../data/worst_case_cost_min_RPI.mat', 'max_cost_milp', 'max_cost_data');
save('./../data/worst_case_cost_min_RPI.mat', 'max_cost_milp');
