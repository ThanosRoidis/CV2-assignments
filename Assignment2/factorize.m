function [ M, S  ] = factorize( pvm, eliminate_affine_ambiguity )
%FACTORIZE It receives a dense measurment matrix and factorizes
%it by returning the camera matrix M and the points matrix S. The second
%arguement indicates whether the affine ambiguity should be eliminated.
    
    pvm = pvm - mean(pvm,2);

    D = pvm;
    
    Xs = D(1:2:end,:);
    Ys = D(2:2:end,:);
%     disp(size(D));
    D = [Xs; Ys];
%     disp(size(D));
    
    [U,W,V] = svd(D);
    
    U = U(:,1:3);
    W = W(1:3,1:3);
    V = V(:,1:3);
    %W is diagonal, i.e. sqrt = sqrtm
    M = U * sqrt(W);
    S  = sqrt(W) * V';
%     M = U;
%     S = W * V';

    if nargin < 2
        eliminate_affine_ambiguity = false;
    end
%     
%     if eliminate_affine_ambiguity == true
%     %%  Eliminate Affine Ambiguity
%         [m, n] = size(pvm);
%         m = m/2;
% 
%         A = cell(m); 
%         for i=1:m
%             A{i} = M(i:(i+1),:);
%             disp(A{i});
%         end 
% 
%         for i=1:m
%             Ai = M(i:(i+1),:);
%             L = pinv(Ai' * Ai);
% 
%     %         L = pinv(A{i}' * A{i});
% 
%             [C, p] = chol(L);
% 
%     %       if L is not positive definite, find the closest positive definite
%     %       matrix
%             if p > 0  
%                 %set the negative eigenvalues of L to zero and recompute L.
%                 [L_V,L_D] = eig(L);  
% 
%                 diagonal = diag(L_D);  
%                 diagonal(diagonal <= 0.001) = 0.001; 
%                 L_D = diag(diagonal);
% 
%                 L = L_V * L_D / L_V;
% 
%                 C = chol(L);  
%             end
% 
% 
%             %if L is positive definite
%             if p == 0
%                 M = M*C';
%                 S = C' \ S;
%             end
% 
%         end
%     end

    if eliminate_affine_ambiguity
        %%% 4. Compute Q, impose the metric constraints
        F = size(M,1)/2;
        Is = M(1:F, :);
        Js = M(F+1:end, :);


        gfun = @(a, b)[ a(1)*b(1), a(1)*b(2)+a(2)*b(1), a(1)*b(3)+a(3)*b(1), ...
                      a(2)*b(2), a(2)*b(3)+a(3)*b(2), a(3)*b(3)] ;
        G = zeros(3*F, 6);
        for f = 1:3*F
            if f <= F
                G(f, :) = gfun(Is(f,:), Is(f,:));
            elseif f <= 2*F
                %        fprintf('do j(%d) ', mod(f, F+1)+1);
                G(f, :) = gfun(Js(mod(f, F+1)+1, :), Js(mod(f, F+1)+1, :));
            else
                %        fprintf('\tdo i,j(%d)', mod(f, 2*F));
                G(f, :) = gfun(Is(mod(f, 2*F),:), Js(mod(f, 2*F),:));
            end
        end

        c = [ones(2*F, 1); zeros(F, 1)];

        % solve Gl = c by SVD and mldivide
        [U S_ V] = svd(G);
        hatl = U'*c;
        y = [hatl(1)/S_(1,1); hatl(2)/S_(2,2); hatl(3)/S_(3,3); hatl(4)/S_(4,4); ...
            hatl(5)/S_(5,5); hatl(6)/S_(6,6)];
        l = V*y;
        fprintf('resid with SVD= Gl - c, %g\n', norm(G*l - c));
        l2 = G\c;
        fprintf('resid with mldivide = Gl - c, %g\n', norm(G*l2 - c));
        % they give the same result because matlab is optimized

        % could be a programatic way, but hey we "see" 3D or 2D
        L = [l(1) l(2) l(3);...
             l(2) l(4) l(5);...
             l(3) l(5) l(6)] ;
        
        [Q,p] = chol(L); % finally!
        
%       if L is not positive definite, find the closest positive definite
%       matrix
        if p > 0  
            %set the negative eigenvalues of L to zero and recompute L.
            [L_V,L_D] = eig(L);  

            diagonal = diag(L_D);  
            diagonal(diagonal <= 0.001) = 0.001; 
            L_D = diag(diagonal);

            L = L_V * L_D / L_V;

            Q = chol(L);  
        end
        
        
        disp(Q)
        M = M * Q;
        S  = Q' \ S ;
        
    end
    
end

