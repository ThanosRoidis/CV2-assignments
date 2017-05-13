function [ F ] = getFundamentalMatrix( matches )
%GETFUNDAMENTALMATRIX Summary of this function goes here
%   Detailed explanation goes here
    
    n = size(matches,1);
    
    %%  Normalize
    T = T_norm(matches(:,1), matches(:,2));
    p = [matches(:,1:2), ones(n,1)];
    p = p*T';
%     disp(mean(p))
%     disp(sum ( p - mean(p)));
%     disp('---');
    
    T_ = T_norm(matches(:,3), matches(:,4));
    q = [matches(:,3:4), ones(n,1)];
    q = q * T_';
    
    matches(:,1:2) = p(:,1:2);
    matches(:,3:4) = q(:,1:2);
    
    %% Construct A
    A = zeros(n, 9);
    A(:,1) = matches(:,1) .* matches(:,3);
    A(:,2) = matches(:,1) .* matches(:,4);
    A(:,3) = matches(:,1);
    A(:,4) = matches(:,2) .* matches(:,3);
    A(:,5) = matches(:,2) .* matches(:,4);
    A(:,6) = matches(:,2);
    A(:,7) = matches(:,3);
    A(:,8) = matches(:,4);
    A(:,9) = ones(n, 1);

    %% Get F through SVD
    [U, D, V] = svd(A); 
    F = V(:,end);
    F = reshape(F, [3,3]);
    
    %% Enforce Singularity on F
    [Uf, Df, Vf] = svd(F);
    Df(end) = 0;
    F = Uf * Df * Vf';

    %% Denormalize
    F = T_' * F * T; 
    
end

