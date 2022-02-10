clear all;
close all;
addpath('./../../auxiliary_funs/');


%% Params
u_ub =  5.0;
u_lb = -5.0;
max_iter = 5;
r_max = 10;


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
n_comb = 3;
Hp = combinator(n_comb, 2, 'p', 'r');        
Hp = (Hp - 1) / (n_comb - 1) * 2 - 1;   % Scale from -1 to 1
Hp = Hp(any(Hp, 2), :);                 % remove all zeros row


%% Algorithm 3
tic;
iter_sets = [];
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


%% Plot result
figure();
X_init = Polyhedron(H{end}, h{end});
plot(X_init, 'color', 'red', 'alpha', 0.1);
hold on;
for i = 1:length(iter_sets)
    plot(iter_sets(i,1), 'color', 'blue', 'alpha', 0.5);
end


%% Save results
Cinv = intersect(iter_sets(iter-1), X);
% invA = Cinv.A;
% invb = Cinv.b;
% H_init = X_init.A;
% h_init = X_init.b;
% H = {};
% h = {};
% for i = 1:length(iter_sets)
%    H_x_{i,1} = iter_sets(i).A;
%    h_x_{i,1} = iter_sets(i).b;
% end
% save('data/iterative_max_RPI.mat', 'invA', 'invb', 'H_init', 'h_init', 'H_x_', 'h_x_');