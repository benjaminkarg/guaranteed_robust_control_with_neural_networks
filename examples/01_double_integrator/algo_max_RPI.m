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
% load('./data/iterative_min_RPI.mat');
% X_s = Polyhedron(H{end}, h{end});
H_s = [eye(2); -eye(2)];
h_s = [0.3; 0.3; 0.3; 0.3];
X_s = Polyhedron(H_s, h_s);
% load('./data/approx_min_RPI_sim_based.mat');
% X_s = Polyhedron(RPI_A, RPI_b);
X_init = X_s.copy();


%% Generate hyperplane directions
n_comb = 3;
Hp = combinator(n_comb, 2, 'p', 'r');        
Hp = (Hp - 1) / (n_comb - 1) * 2 - 1;   % Scale from -1 to 1
Hp = Hp(any(Hp, 2), :);                 % remove all zeros row


%% Algorithm 3
tic;
iter_sets = [X_s];
d_worst_case = compute_d_worst_case(D, Hp, A, E);
for iter = 1:max_iter
   
    % Compute Preimage
%     X_pre = preimage(network, X_s, D, Hp, A, B, E);
    X_pre = preimage_worst_case(network, X_s, d_worst_case, Hp, A, B, E);
%     figure(); plot(X_pre,'color','blue','alpha',0.5); hold on; plot(X_pre_2,'color','red','alpha',0.5);
    X_pre = intersect(X_pre, X);
    
    % Check if preimage 'r'-step admissible positive invariant
    [r, sets, success] = r_step_invariance(network, Hp, X, X_pre, D, r_max, A, B, E);
%     success = true;
    save_str = strcat('data/algo_max_RPI_iter_', num2str(iter), '.mat');
    if success
        H = {};
        h = {};
        for i = 1:length(sets)
            H{i,1} = sets(i).A;
            h{i,1} = sets(i).b;
        end
        save(save_str, 'H', 'h');
    end
    
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
plot(X_init, 'color', 'red', 'alpha', 0.1);
hold on;
for i = 1:length(iter_sets)
    plot(iter_sets(i,1), 'color', 'blue', 'alpha', 0.5);
end


%% Save results
Cinv = intersect(iter_sets(end), X);
iter_sets = horzcat(iter_sets, Cinv);
H = {};
h = {};
for i = 1:length(iter_sets)
   H{i,1} = iter_sets(i).A;
   h{i,1} = iter_sets(i).b;
end
save('data/iterative_max_RPI.mat', 'H', 'h', 'comp_time');