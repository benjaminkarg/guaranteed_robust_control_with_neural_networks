function [Pre] = preimage(network_controller, X_s, D, Hp, A, B, E)
% network controller must satisfy the control input constraints
% Hp: Direction of considered hyperplanes: n_hp x nx
% X_s is the set for which the over-approximation of the preimage is
% computed
% D: Polyhedron containing the disturbances
% Hp are the directions of hyperplanes that are considered for the computation of the preimage
% System matrices: x_plus = A * x + B * u + E * d

% Extract weights and biases from network controller
weights = network_controller.weights;
biases = network_controller.biases;

% sizes
nx = size(A, 2);
nu = size(B, 2);
nd = size(E, 2);

% big-M
M = 1e5;

% construct problem
cons = [];
x0 = sdpvar(nx,1);
cons = cons + [-1e6 <= x0 <= 1e6];
t = {};
z = {};
z{1,1} = sdpvar(nx,1);
cons = cons + [x0 == z{1,1}]; % (25d)

for i = 1:length(weights)-1
    [n_neurons] = size(weights{1,i},1);
    z{1,i+1} = sdpvar(n_neurons,1);
    t{1,i} = binvar(n_neurons,1); % (25j)
    cons = cons + [z{1,i+1} >= weights{1,i} * z{1,i} + biases{1,i}]; %(25f)
    cons = cons + [z{1,i+1} <= weights{1,i} * z{1,i} + biases{1,i} + M * t{1,i}]; % (25g)
    cons = cons + [0 <= z{1,i+1}]; % (25h)
    cons = cons + [z{1,i+1} <= M -  M * t{1,i}]; % (25i)
end

u0 = sdpvar(nu,1);
cons = cons + [u0 == weights{1,end} * z{1,end} + biases{1,end}]; % (25k)

d0 = sdpvar(nd,1);
cons = cons + [D.A * d0 <= D.b];

x1 = sdpvar(nx,1);
cons = cons + [A * x0 + B * u0 + E * d0  - x1 == 0 ]; % (25c)
cons = cons + [X_s.A * x1 <= X_s.b];
ops = sdpsettings('solver', 'gurobi');

b_max = zeros(size(Hp,1),1);
for i = 1:size(Hp,1)
    obj = -Hp(i,:) * x0;
    optimize(cons, obj, ops);
    b_max(i,1) = -value(obj);
end

Pre = Polyhedron(Hp, b_max);

end
