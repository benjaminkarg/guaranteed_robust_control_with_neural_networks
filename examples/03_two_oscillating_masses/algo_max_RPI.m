clear all;
close all;
addpath('./../../auxiliary_funs/');


%% Params
u_ub =  5.0;
u_lb = -5.0;
max_iter = 5;
r_max = 5;


%% Load the neural network
load('./data/nn_controller.mat');
network = make_network_input_admissible(network, u_lb, u_ub);


%% Load system
load('./data/system_and_problem_matrices.mat');


%% Sets
% Admissible state space
X = Polyhedron(H_x, h_x);

% Disturbance set
D = Polyhedron(H_d, h_d);

% Initial set (min RPI)
load('./data/iterative_min_RPI.mat');
X_s = Polyhedron(H{end}, h{end});


%% Generate hyperplane directions
nx = size(H_x, 2);
n_comb = 5;
Hp = combinator(n_comb, nx, 'p', 'r');        
Hp = (Hp - 1) / (n_comb - 1) * 2 - 1;   % Scale from -1 to 1
Hp = Hp(any(Hp, 2), :);                 % remove all zeros row


%% Algorithm 3
tic;
iter_sets = [X_s];
for iter = 1:max_iter
   
    % Compute Preimage
    X_pre = preimage(network, X_s, D, Hp, A, B, E);
    
    % Check if preimage 'r'-step admissible positive invariant
    [r, sets, success] = r_step_invariance(network, Hp, X, X_pre, D, r_max, A, B, E);
    
    % Update if admissible, otherwise break
    if success
        iter_sets = horzcat(iter_sets, X_pre);
        X_s = X_pre;
    else
         break;
    end
    
end
comp_time = toc;


%% Save results
Cinv = intersect(iter_sets(end), X);
iter_sets = horzcat(iter_sets, Cinv);
H = {};
h = {};
for i = 1:length(iter_sets)
   H{i,1} = iter_sets(i).A;
   h{i,1} = iter_sets(i).b;
end
save('data/iterative_max_RPI.mat', 'H', 'h');