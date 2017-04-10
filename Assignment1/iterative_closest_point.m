%% iterative closest point
function [R,t] = iterative_closest_point(A1, A2)
    %% Initialize R = I, t = 0
    R = eye(3);
    t = zeros(1, 3);
    A1_transf = A1 * R + t;

%% Find the closest points for each point in the base point set (A1) from the
% target point set (A2) using brute-force approach.
    for i=1:40
        pairwise_dist = pdist2(A1_transf, A2);
        [~, index] = min(pairwise_dist, [], 2);
        psi_A1 = A2(index, :);

        [R,t] = svd_rot(A1, psi_A1);
        %%
        A1_transf = A1 * R + t;

        %dist = RMS(A1, psi_A1);
    end
    
end

%% Root Mean Square
function dist = RMS(A1_transf, psi_A1)
    dist = sum(sum(((A1_transf-psi_A1).^2), 2));
end