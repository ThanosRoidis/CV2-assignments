function [ M, S ] = factorize( pvm, eliminate_affine_ambiguity )
%UNTITLED It receives a dense block of the pointview matrix and factorizes
%it
    
    pvm = pvm - mean(pvm,2);

    D = pvm;
    
    [U,W,V] = svd(D);
    
    U = U(:,1:3);
    W = W(1:3,1:3);
    V = V(:,1:3);
    %W is diagonal, i.e. sqrt = sqrtm
    M = U * sqrt(W);
    S = sqrt(W) * V';
%     M = U;
%     S = W * V';

    if nargin < 2
        eliminate_affine_ambiguity = false;
    end
    
    if eliminate_affine_ambiguity == true
    %%  Eliminate Affine Ambiguity
        [m, n] = size(pvm);
        m = m/2;

        A = cell(m); 
        for i=1:m
            A{i} = M(i:(i+1),:); 
        end 

        for i=1:m
            Ai = M(i:(i+1),:);
            L = pinv(Ai' * Ai);

    %         L = pinv(A{i}' * A{i});

            [C, p] = chol(L);

    %       if L is not positive definite, find the closest positive definite
    %       matrix
            if p > 0  
                %set the negative eigenvalues of L to zero and recompute L.
                [L_V,L_D] = eig(L);  

                diagonal = diag(L_D);  
                diagonal(diagonal <= 0.001) = 0.001; 
                L_D = diag(diagonal);

                L = L_V * L_D / L_V;

                C = chol(L);  
            end


            %if L is positive definite
            if p == 0
                M = M*C';
                S = C' \ S;
            end

        end
    end
    
end

