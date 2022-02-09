function [max_cost] = max_stage_cost(network, min_RPI, Q, R, A, B, E)
% network: network controller, must be input-admissible
% min_RPI: Polyhedron describing the approximate minimum robust positive invariant set
% Q: stage cost on states, nx x nx
% R: stage cost on inputs, nu x nu
% A, B, E: system matrices from x_plus = A * x + B * u + E * d

% Defaul values
success = false;
M = 1e5; % probably high enough value, should be computed by considering the corresponding NN

% Unpack network
weights = network.weights;
biases = network.biases;
nu = size(biases{1,end},1);
nx = size(min_RPI.A,2);

% initial set to check
invA = min_RPI.A;
invb = min_RPI.b;

% Formulate problem with YALMIP
cons = [];
x0 = sdpvar(nx,1);
cons = cons + [invA * x0 <= invb];
t = {};
z = {};
z{1,1} = sdpvar(nx,1);
cons = cons + [x0 == z{1,1}]; % (25d)

for i = 1:length(weights)-1
    n_neurons = size(weights{1,i},1);
    z{1,i+1} = sdpvar(n_neurons,1);
    t{1,i} = binvar(n_neurons,1); % (25j)
    cons = cons + [z{1,i+1} >= weights{1,i} * z{1,i} + biases{1,i}]; %(25f)
    cons = cons + [z{1,i+1} <= weights{1,i} * z{1,i} + biases{1,i} + M * t{1,i}]; % (25g)
    cons = cons + [0 <= z{1,i+1}]; % (25h)
    cons = cons + [z{1,i+1} <= M -  M * t{1,i}]; % (25i)
end

u0 = sdpvar(nu, 1);
cons = cons + [u0 == weights{1,end} * z{1,end} + biases{1,end}]; % (25k)

ops = sdpsettings('solver', 'gurobi');

% next iteration
obj = - x0' * Q * x0 - u0' * R * u0;
optimize(cons, obj, ops);
max_cost = -value(obj);

end
