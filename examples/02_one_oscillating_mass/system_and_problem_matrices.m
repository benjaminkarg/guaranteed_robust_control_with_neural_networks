clear all;
close all;

%% Params
compute_MRCI = true;
saveloc = './data/';
iter_max = 1000;
roh = 0.2;


%% Continous-time system matrices
ts = 0.5; % sampling time for discretization
c = 1.0; % spring constant
m = 1.0; % mass
A_cont = [0, 1; -2 * c/m, 1/m];
B_cont = [0; 1/m];
nx = size(A_cont,2);
nu = size(B_cont,2);


%% Conversion to discrete-time system matrices
sys = ss(A_cont, B_cont, eye(nx), zeros(size(B_cont)));
sysd = c2d(sys, ts);
A = sysd.A;
B = sysd.B;
E = eye(nx);


%% Admissible state space
H_x = [eye(nx); -eye(nx)];
h_x = repmat([4.0; 2.0], 2, 1);
X = Polyhedron(H_x, h_x);


%% Admissible control input space
H_u = [eye(nu); -eye(nu)];
h_u = [5; 5];
U = Polyhedron(H_u, h_u);


%% Disturbance and approximation error set
% Disturbance
H_d = [eye(nx); -eye(nx)];
h_d = ones(4,1)*0.1;
D = Polyhedron(H_d, h_d);

% Approximation error learning
H_delta = [ 1; -1];
% h_delta = [1.0; 1.0];
h_delta = [0.05; 0.05];
Delta = Polyhedron(H_delta, h_delta);
X_delta = affineMap(Delta, B); % transform from input to state space

% Combined uncertainty
W = plus(D, X_delta);
H_w = W.A;
h_w = W.b;


%% Matrices of quadratic objective function
Q = eye(nx);
R = eye(nu) * 0.1;


%% Compute LQR feedback
[K, S, e] = dlqr(A, B, Q, R);


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