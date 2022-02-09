function [r, sets, success] = r_step_invariance(network, Hp, X, X_s, D, r_max, A, B, E)
% network controller must be input-admissible
% X: Polyhedron describing the admissible state space
% X_s: Polyhedron describing the set in the state space to be verified
% D: Polyhedron describing the disturbances
% r_max: maximum number of steps to consider for $r$-step invariance
% A, B, E: system matrices from x_plus = A * x + B * u + E * d

% Defaul values
success = false;
r = 0;
M = 1e5; % probably high enough value, should be computed by considering the corresponding NN
sets = [X_s];

% Unpack network
weights = network.weights;
biases = network.biases;
nu = size(biases{1,end},1);
nx = size(X_s.A,2);

% initial set to check
invA = X_s.A;
invb = X_s.b;

% Formulate problem with YALMIP
cons = [];
x0 = sdpvar(nx,1);
cons = cons + [X.A * x0 <= X.b];
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

u0 = sdpvar(nu,1);
cons = cons + [u0 == weights{1,end} * z{1,end} + biases{1,end}]; % (25k)

d0 = sdpvar(size(D.A,2),1);
cons = cons + [D.A * d0 <= D.b];

x1 = sdpvar(nx,1);
cons = cons + [A * x0 + B * u0 + E * d0  - x1 == 0 ]; % (25c)
ops = sdpsettings('solver', 'gurobi');

% Solve the problem
intermediate_sets = [X_s];
for r_iter = 1:r_max

    % next iteration
    b_max = zeros(size(Hp,1),1);
    for i = 1:size(Hp,1)
        obj = -Hp(i,:) * x1;
        optimize(cons, obj, ops);
        b_max(i,1) = -value(obj);
    end
    % Check if resulting set belongs to feasible state space
    X_new = Polyhedron(Hp, b_max);
    eps = 1e-3;
    X_new_eps = Polyhedron(Hp, b_max * (1.0 + eps));
    if ~X.contains(X_new)
        success = false;
        r = r_iter;
        intermediate_sets = horzcat(intermediate_sets, X_new);
        sets = intermediate_sets;
        disp('No admissible r-step invariant set found!');
        break
    end
    % break if feasible set found
    if X_s.contains(X_new_eps)
        success = true;
        disp('Admissible r-step invariant set found!');
        r = r_iter;
        intermediate_sets = horzcat(intermediate_sets, X_new);
        sets = intermediate_sets;
        break
    else
        % save intermediate results
        intermediate_sets = horzcat(intermediate_sets, X_new);
        sets = intermediate_sets;
        % Update constraints
        cons(2) = [Hp * x0 <= b_max];
    end

end

end
