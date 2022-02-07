%% Initiate m-file
clear all;
close all;


%% Params
compute_MRCI = true;
saveloc = './data/';
iter_max = 3;
roh = 0.2;



%% Continous-time system matrices
c = 1.0; % spring constant
m = 1.0; % mass
ts = 0.5; % sampling time

A_cont = [0, 1, 0, 0;
          -2*c/m, 0, c/m, 0;
          0, 0, 0, 1;
          c/m, 0, -2*c/m, 0];

B_cont = [0,   0;
          1/m, 0;
          0,   0;
          0, 1/m];
nx = size(A_cont,2);
nu = size(B_cont,2);


%% Conversion to discrete-time system matrices
sys = ss(A_cont, B_cont, eye(size(A_cont,1)), zeros(size(B_cont)));
sysd = c2d(sys, ts, 'zoh');
A = sysd.A;
B = sysd.B;
E = eye(nx);


%% Admissible state space
H_x = [eye(nx); -eye(nx)];
h_x = repmat([4.0; 2.0], nx, 1);


%% Admissible control input space
H_u = [eye(nu); -eye(nu)];
h_u = 5.0 * ones(2 * nu, 1);
U = Polyhedron(H_u, h_u);


%% Disturbance and approximation error set
% Disturbance
H_d = [eye(nx); -eye(nx)];
h_d = ones(2 * nx, 1) * 0.1;
D = Polyhedron(H_d, h_d);

% Approximation error learning
H_delta = [eye(nu); -eye(nu)];
h_delta = ones(2 * nu, 1) * 0.05;
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