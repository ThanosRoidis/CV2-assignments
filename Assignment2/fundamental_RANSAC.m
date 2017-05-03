function [ bestF ] = fundamental_RANSAC( matches, n, max_iter )
%FUNDAMENTAL_RANSAC Summary of this function goes here
%   Detailed explanation goes here

    n = 8;
    bestF = eye(3);
    most_inliers = 0;
    threshhold = 5;
    
    for i = 1:max_iter
        n = 8;
        r = randsample(size(matches,1),n);
        sampled = matches(r,:);
    
        F = getFundamentalMatrix(sampled);
        
        inliers = 0;
        for j=1:size(matches,1)
            d = sampson_dist(matches(j,:), F);
            
            if d < threshhold
                inliers = inliers + 1;
            end
        end
        
        if inliers > most_inliers
            bestF = F;
        end
        
            
    end



end

