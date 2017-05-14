function [ bestF, most_inliers] = fundamental_RANSAC(matches, n, threshold, max_iter, verbose )
%FUNDAMENTAL_RANSAC Calculates the fundamental matrix F by using RANSAC on
%the normalized 8-point algorithm. 
    %The number of points 'n' used for the normalized 8-point algorithm,
    %the threshold for an inlier, and the number of iterations of RANSAC
    %must be specified. For the distance between an inlier pair, Sampson
    %Distane is used.

    
    if nargin < 5
        verbose = false;
    end

    bestF = eye(3); 
    most_inliers = [];
    
    %Run RANSAC
    for i = 1:max_iter
        %Sample n points for  the normalized 8-point algorithm
        r = randsample(size(matches,1),n);
        sampled = matches(r,:);
        
        F = normalized_8point(sampled);
        
        %Find all the inliers using the Sampson distance
        inliers = [];
        for j=1:size(matches,1)
            d = sampson_dist(matches(j,:), F);
            
            if d < threshold 
                inliers(end + 1,:) = j;
            end
        end
        
        %Update inliers and F, if the number of inliers exceed the already
        %best
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

