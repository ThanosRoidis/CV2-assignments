function [ bestF, most_inliers] = fundamental_RANSAC(matches, n, threshold, max_iter, verbose )
%FUNDAMENTAL_RANSAC Summary of this function goes here
%   Detailed explanation goes here
    
    if nargin < 5
        verbose = false;
    end

    bestF = eye(3); 
    most_inliers = [];
    
    for i = 1:max_iter
        r = randsample(size(matches,1),n);
        sampled = matches(r,:);
        
        F = getFundamentalMatrix(sampled);
        
        inliers = [];

        for j=1:size(matches,1)
            d = sampson_dist(matches(j,:), F);
            
            if d < threshold 
                inliers(end + 1,:) = j;
            end
        end
        
        if size(inliers,1) > size(most_inliers,1)
            bestF = F;
            most_inliers = inliers;
        end
        
        if verbose
            fprintf('Inliers at iteration %d: %d\n', i,  size(inliers,1));
        end
    end

    most_inliers = matches(most_inliers, :);
    if verbose
        fprintf('Maximum inliers: %d\n', size(most_inliers,1));
    end
end

