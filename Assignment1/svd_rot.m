function [ R, t ] = svd_rot( p, q, w )
%SVD_ROT Summary of this function goes here
%   Detailed explanation goes here
    N = size(p,1);
    d = size(p,2);
    
    if nargin == 2
        w = ones(N, 1);
    end
    
    p_ = mean(w.*p, 1);
    
    p_ = p_ * N / sum(w);
    
    q_ = mean(w.*q, 1);
    q_ = q_ * N / sum(w);
    
    X = p - p_;
    Y = q - q_;
    
    S = X' * diag(w) * Y;

    [U, ~, V] = svd(S);
    
    eye_det = eye(d);
    eye_det(d,d) = det(V * U');
    
    R = V * eye_det * U';
    R = R';
    
    t = q_ - p_* R;
end

