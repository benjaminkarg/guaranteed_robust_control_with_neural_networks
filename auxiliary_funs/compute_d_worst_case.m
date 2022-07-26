function [d_worst_case] = compute_d_worst_case(D, Hp, A, E)
% Hp: Direction of considered hyperplanes: n_hp x nx
% D: Polyhedron containing the disturbances
% Hp are the directions of hyperplanes that are considered for the computation of the preimage
% System matrices: x_plus = A * x + B * u + E * d

% sizes
nx = size(A, 2);
nd = size(E, 2);

% construct problem
cons = [];
d0 = sdpvar(nd,1);
cons = cons + [D.A * d0 <= D.b];
ops = sdpsettings('solver', 'gurobi');

Ainv = inv(A);
d_worst_case = zeros(size(Hp,1),nd);
for i = 1:size(Hp,1)
    obj = -Hp(i,:) * Ainv * E * d0;
    optimize(cons, obj, ops);
    d_worst_case(i,:) = value(d0)';
end

end
