function Fas = approx_mRPI(A, B, K, H_w, h_w, eps, s_max)

    
    % closed-loop
    A = A - B * K;
    
    
    % disturbance set
    W = Polyhedron(H_w, h_w);
    
   
    % Algorithm 1 from Rakovic 2005
    for s = 1:s_max
    
        % (9), (11)
        As = A^s;
        sup_w = [];
        for i = 1:size(W.A,1)
            fi = W.A(i,:);
            a = As' * fi';
            supp = W.support(a);
            sup_w(i) = supp/W.b(i);
        end

        alpha = max(sup_w);

        % (13)
        Msj = [];
        for j = 1:size(W.A,2)

            % standard basis vector
            ej = zeros(size(W.A,2),1);
            ej(j) = 1;

            %
            pos_sum = 0;
            neg_sum = 0;
            for i = 0:s-1

               Ai = A^i;
               Aij = Ai' * ej;
               % positive sum
               pos_add = W.support(Aij);
               pos_sum = pos_sum + pos_add;

               % negative sum
               neg_add = W.support(-Aij);
               neg_sum = neg_sum + neg_add;
            end

            Msj(j) = max(pos_sum, neg_sum);

        end

        Ms = max(Msj);

        % break condition
        cond = eps/(eps + Ms);
        if alpha <= cond
            disp('Converged');
            disp(s);
            break;
        end

    end
    
    % Compute the epsilon approximation of the mRPI (2)
    Fs = W.copy(); % equal to Fs with s = 1;
    for ss = 2:s
       add_minkowski = affineMap(W, A^(ss-1));
       Fs = plus(Fs, add_minkowski);
    end
    mul_fac = (1 - alpha)^(-1);
    Fas = Polyhedron(Fs.A, mul_fac * Fs.b);

end
