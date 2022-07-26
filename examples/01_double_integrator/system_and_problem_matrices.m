clear all;
close all;

%% Params
compute_MRCI = true;
saveloc = './data/';
iter_max = 1000;
roh = 0.01;


%% System matrices
A = [1, 1; 0, 1];
B = [0.5; 1];
E = eye(2);


%% Admissible state space
H_x = [eye(2); -eye(2)];
h_x = ones(4,1) * 5;
X = Polyhedron(H_x, h_x);


%% Admissible control input space
H_u = [1; -1];
h_u = [1; 1];
U = Polyhedron(H_u, h_u);


%% Disturbance and approximation error set
% Disturbances
H_d = [eye(2); -eye(2)];
h_d = ones(4,1) * 0.1;
D = Polyhedron(H_d, h_d);

% Approximation error learning
H_delta = [ 1; -1];
h_delta = [0.02; 0.02];
Delta = Polyhedron(H_delta, h_delta);
X_delta = affineMap(Delta, B); % transform from input to state space

% Combined uncertainty
W = plus(D, X_delta);
H_w = W.A;
h_w = W.b;


%% Matrices of quadratic objective function
Q = eye(2);
R = 1;


%% Compute things
MRCIs = {n_combs, 1};
Ks = {};:

for i = 1:length()

    %% Compute LQR feedback
    [K, S, e] = dlqr(A, B, Q, R);


    
    
end


%% Save resulting system matrices and disturbance set
if not(isfolder('data'))
    mkdir('data')
end
save('data/system_and_problem_matrices.mat', 'A', 'B', 'E', 'Q', 'R', 'K', 'S', 'H_x', 'h_x', 'H_u', 'h_u', 'H_d', 'h_d', 'H_delta', 'h_delta', 'H_w', 'h_w');


%% Compute MRCI
if compute_MRCI
    addpath('../../auxiliary_funs/');
    compute_MRCI_rungger_tabuada('data/system_and_problem_matrices.mat', saveloc, iter_max, roh)
end