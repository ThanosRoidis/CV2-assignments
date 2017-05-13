clear;
clc;

matches_pairs = load('matches_pairs.mat');
matches_pairs = matches_pairs.matches_pairs;

pointview_matrix = [];

for i=1 : 3%size(matches_pairs, 2)
    
    % get inliers for frame i, i+1
    [~, inliers] = fundamental_RANSAC(matches_pairs{i}, 8, 1, 100, false);
    inliers = round(inliers);
    
    indices = [];
    
    % Eliminating detected interest points on background
    for inlier=1 : size(inliers, 1)
        frame_str = sprintf('House/frame%08d.png', i);
        frame= imread(frame_str);
        
        if frame(inliers(inlier,2),inliers(inlier,1)) < 25
            indices=[indices, inlier];
        end
        
    end
    inliers(indices,:)= [];
    %
    
    if i==1
        
        for p=1 :size(inliers, 1)
            
            pointview_matrix(i, p) = inliers(p,1);
            pointview_matrix(i+1, p) = inliers(p,2);
            
            pointview_matrix(i+2, p) = inliers(p,3);
            pointview_matrix(i+3, p) = inliers(p,4);
            
        end
        
    else
        ii = 2* i - 1;
        for p=1 :size(inliers, 1)
            
            % coordinates for match point in image i
            x_current = inliers(p,1);
            y_current = inliers(p,2);
            
            
            % check if this match already exists
            match_already_exists = 0;
            match_id = -1;
            for match=1 :size(pointview_matrix, 2) % n
                if( x_current == pointview_matrix(ii, match) &&...
                        y_current ==  pointview_matrix(ii+1, match))
                    match_already_exists = 1;
                    match_id = match;
                    break;
                end
            end
            
            if match_already_exists == 1
                % do not have to add new column since match already exists
                
                pointview_matrix(ii+2, match_id) = inliers(p,3);
                pointview_matrix(ii+3, match_id) = inliers(p,4);
                
            else
                % add new column for this particular point
                
                pointview_matrix(ii, end+1) = inliers(p,1);
                pointview_matrix(ii+1, end) = inliers(p,2);
                
                pointview_matrix(ii+2, end) = inliers(p,3);
                pointview_matrix(ii+3, end) = inliers(p,4);
            end
            
            
        end
        
    end
    
end

% create measurement matrix and visualize 
measurement_matrix = pointview_matrix(1:2:end,:);  % odd matrix
measurement_matrix = spones(measurement_matrix);
imagesc(measurement_matrix);
 %imshow(m);
