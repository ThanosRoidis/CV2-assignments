function [ bestF ,best_inlier_points] = fundamental_RANSAC( matches, n, max_iter )
%FUNDAMENTAL_RANSAC Summary of this function goes here
%   Detailed explanation goes here

%     n = 8;
    bestF = eye(3);
    most_inliers = 0;
    threshhold = 1.0e-04;%5;
    best_inlier_points = [];
    
    for i = 1:max_iter
%         n = 8;
        r = randsample(size(matches,1),n);
        sampled = matches(r,:);
        
        F = getFundamentalMatrix(sampled);
        
        inliers = 0;
        inlier_points = [];

        for j=1:size(matches,1)
            d = sampson_dist(matches(j,:), F);
            
            if d < threshhold
                inliers = inliers + 1;
                inlier_points = [inlier_points; matches(j,:)];

            end
        end
        
        if inliers > most_inliers
            bestF = F;
            best_inlier_points = inlier_points;
        end
        
            
    end



end

