function [X_r] = comp_r_step_set(network, Hp, X_s, D, r, A, B, E)
% network controller must be input-admissible
% X: Polyhedron describing the admissible state space
% X_s: Polyhedron describing the set in the state space to be verified
% D: Polyhedron describing the disturbances
% r_max: maximum number of steps to consider for $r$-step invariance
% A, B, E: system matrices from x_plus = A * x + B * u + E * d

% Defaul values
M = 1e5; % probably high enough value, should be computed by considering the corresponding NN

% Unpack network
weights = network.weights;
biases = network.biases;
nu = size(biases{1,end},1);
nx = size(X_s.A,2);

% Formulate problem with YALMIP
cons = [];
x = {};
u = {};
d = {};
t = {};
z = {};

% Initial condition
x{1} = sdpvar(nx,1);
cons = cons + [X_s.A * x{1} <= X_s.b];


for j = 1:r
    
    z{j,1} = sdpvar(nx,1);
    cons = cons + [x{j} == z{j,1}]; % (25d)

    for i = 1:length(weights)-1
        n_neurons = size(weights{1,i},1);
        z{j,i+1} = sdpvar(n_neurons,1);
        t{j,i} = binvar(n_neurons,1); % (25j)
        cons = cons + [z{j,i+1} >= weights{1,i} * z{j,i} + biases{1,i}]; %(25f)
        cons = cons + [z{j,i+1} <= weights{1,i} * z{j,i} + biases{1,i} + M * t{j,i}]; % (25g)
        cons = cons + [0 <= z{j,i+1}]; % (25h)
        cons = cons + [z{j,i+1} <= M -  M * t{j,i}]; % (25i)
    end
    
    % Construct control input
    u{j} = sdpvar(nu,1);
    cons = cons + [u{j} == weights{1,end} * z{j,end} + biases{1,end}]; % (25k)

    % Construct uncertainty/disturbance
    d{j} = sdpvar(size(D.A,2),1);
    cons = cons + [D.A * d{j} <= D.b];
    
    % System equation
    x{j+1} = sdpvar(nx,1);
    cons = cons + [A * x{j} + B * u{j} + E * d{j}  - x{j+1} == 0 ]; % (25c)
    
end

% solver settings
ops = sdpsettings('solver', 'gurobi');

% Solve the problem
b_max = zeros(size(Hp,1),1);
for i = 1:size(Hp,1)
    obj = -Hp(i,:) * x{end};
    optimize(cons, obj, ops);
    b_max(i,1) = -value(obj);
end
% Check if resulting set belongs to feasible state space
X_r = Polyhedron(Hp, b_max);

end
