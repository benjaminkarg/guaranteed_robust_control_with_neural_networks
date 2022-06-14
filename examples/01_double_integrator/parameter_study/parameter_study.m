clear all;
close all;
addpath('../../../auxiliary_funs/');


%% Params
u_ub =  1.0; % lower bound of control input
u_lb = -1.0; % upper bound of control input
% n_hp = [10, 20, 40, 80]; % number of hyperplanes
r_seqs = {[1, 1, 1, 1, 1, 1, 1, 1], [2, 2, 2, 2], [4, 4], [8]}; % r sequences


%% Load the neural network
load('./../data/nn_controller.mat');
network = make_network_input_admissible(network, u_lb, u_ub);


%% Load system
load('./../data/system_and_problem_matrices.mat');


%% Load and create sets for computations
load('./../data/MRCI.mat');
X_s = Polyhedron(MRCI_A, MRCI_b);
% Initial set
% H = [ 1,  0;
%      -1,  0;
%       0,  1;
%       0, -1];
%   
% h = [1.5; 1.5; 1.5; 1.5];
% X_s = Polyhedron(H, h);

% disturbance set
D = Polyhedron(H_d, h_d);

% Compute the sets
for k = 1:length(n_hp)
    
    % Data containers
    areas = zeros(length(r_seqs), 1);
    t_comps = zeros(length(r_seqs), 1);
    Rs = {};
    rs = {};
    
    
    % Fix random seed to obtain identical hyperplanes
    rng(13209);
    Hp = rand(2, n_hp(k))' * 2 - 1;

    
    for j = 1:length(r_seqs)
        
        % initial set
        sets = {X_s};
        
        %% Compute r-step reachable set sequentially
        tic;
        for i = 1:length(r_seqs{j})
            X_r = comp_r_step_set(network, Hp, sets{end}, D, r_seqs{j}(i), A, B, E);
            sets{i+1} = X_r;
        end
        comp_time = toc;
        
        %% Plot result
        figure();
        hold on;
        % Plot sets of the iteration (starting with the set to be verified)
        for i = 1:length(sets)
            plot(sets{i}, 'color', 'blue', 'alpha', 0.1);
        end
        
        % Update data
        areas(j, 1) = sets{end}.volume;
        t_comps(j,1) = comp_time;
        Rs{j} = sets{end}.A;
        rs{j} = sets{end}.b;

    end
    
    % Save results
    savename = strcat('./', num2str(n_hp(k)), '.mat');
    save(savename, 'areas', 't_comps', 'Rs', 'rs');

end

