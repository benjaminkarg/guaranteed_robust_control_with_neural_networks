function [h] = polyhedral_approx(X, H)
    % X: data points: n_samples x n_x
    % H: direction of hyperplanes: n_hp x n_x
    
    % sizes
    n_hp = size(H, 1);
    n_s = size(X,1);

    % Compute static elements of optimization problem
    f = ones(n_hp, 1);
    A_ineq = -eye(n_hp);
    lb = zeros(n_hp, 1);
    ub = ones(n_hp, 1) * inf;

    % initialize data
    h_coll = zeros(n_hp, n_s);
    
    parfor i = 1:size(X,1)
       x_cur = X(i,:)';
       b_ineq = H * x_cur;
       res = linprog(f, A_ineq, b_ineq, [], [], lb, ub);
       h_coll(:, i) = res;
    end
    h = max(h_coll, [], 2);
end