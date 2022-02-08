%% Initiate m-file
clear all;
close all;


%% Continous-time system matrices
c = 1.0; % spring constant
m = 1.0; % mass
ts = 0.5; % sampling time
n_masses = 3;
a_uneven = zeros(1, n_masses * 2);
a_uneven(1,2) = 1;

A_cont = zeros(n_masses * 2, n_masses * 2);
for i = 1:n_masses
    A_cont((i-1) * 2 + 1,:) = a_uneven;
    a_uneven = circshift(a_uneven, 2, 2);
    if i == 1
        A_cont(2, 1) = - (2 * c / m);
        A_cont(2, 3) = c /m ;
    elseif i == n_masses
        A_cont(2 * n_masses, end - 1) = - (2 * c / m);
        A_cont(2 * n_masses, end - 3) = c / m;
    else
       A_cont(i * 2, (i-2) * 2 + 1) = c / m;
       A_cont(i * 2, (i-1) * 2 + 1) = - (2 * c / m);
       A_cont(i * 2,  i    * 2 + 1) = c / m;
    end
end


b_unit = [0; 1];
B_cont = blkdiag(b_unit, b_unit, b_unit);

%% Conversion to discrete-time system matrices
sys = ss(A_cont, B_cont, eye(size(A_cont,1)), zeros(size(B_cont)));
sysd = c2d(sys, ts, 'zoh');
A = sysd.A;
B = sysd.B;
E = eye(2 * n_masses);
nx = size(A,2);
nu = size(B,2);
nw = size(E,2);


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
W.minHRep;
H_w = W.A;
h_w = W.b;


%% Matrices of quadratic objective function
Q = eye(nx);
R = eye(nu) * 0.1;


%% Compute LQR feedback
[K, S, e] = dlqr(A_cont, B_cont, Q, R);


%% Save resulting system matrices and disturbance set
if not(isfolder('data'))
    mkdir('data')
end
savestr = strcat('data/system_and_problem_matrices.mat');
save('data/system_and_problem_matrices.mat', 'A', 'B', 'E', 'Q', 'R', 'K', 'S', 'H_x', 'h_x', 'H_u', 'h_u', 'H_d', 'h_d', 'H_delta', 'h_delta', 'H_w', 'h_w');
