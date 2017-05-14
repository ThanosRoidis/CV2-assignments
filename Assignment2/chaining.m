function [measurement_matrix, one_hot_matrix] = chaining(matches_pairs)
%CHAINING Given a cell array of matches, in which each element contains the
%raw matches between a pair of views (as given by vl_ubcmatch), it
%calculates the sparse measurement matrix by keeping only the inliers from the normalized 8-point algorithm.

    measurement_matrix = [];

    for i=1 : size(matches_pairs, 2)

        % get inliers for frame i, i+1
        [~, inliers] = fundamental_RANSAC(matches_pairs{i}, 8, 1, 20, false);
        inliers = round(inliers);

    %     indices = [];
    %     
    %     % Eliminating detected interest points on background
    %     for inlier=1 : size(inliers, 1)
    %         frame_str = sprintf('House/frame%08d.png', i);
    %         frame= imread(frame_str);
    %         
    %         if frame(inliers(inlier,2),inliers(inlier,1)) < 25
    %             indices=[indices, inlier];
    %         end
    %         
    %     end
    %     inliers(indices,:)= [];
        %

        if i==1
            measurement_matrix = inliers';
        else
            ii = 2* i - 1;
            for p=1 :size(inliers, 1)

                % coordinates for match point in image i
                x_current = inliers(p,1);
                y_current = inliers(p,2);


                % check if this match already exists
                match_already_exists = 0;
                match_id = -1;
                for match=1 :size(measurement_matrix, 2) % n
                    if( x_current == measurement_matrix(ii, match) &&...
                            y_current ==  measurement_matrix(ii+1, match))
                        match_already_exists = 1;
                        match_id = match;
                        break;
                    end
                end

                if match_already_exists == 1
                    % do not have to add new column since match already exists

                    measurement_matrix(ii+2, match_id) = inliers(p,3);
                    measurement_matrix(ii+3, match_id) = inliers(p,4);

                else
                    % add new column for this particular point

                    measurement_matrix(ii, end+1) = inliers(p,1);
                    measurement_matrix(ii+1, end) = inliers(p,2);

                    measurement_matrix(ii+2, end) = inliers(p,3);
                    measurement_matrix(ii+3, end) = inliers(p,4);
                end


            end

        end

    end

    %create one-hot matrix (1 = point exists, 0 otherwise)
    one_hot_matrix = measurement_matrix(1:2:end,:);  % odd matrix
    one_hot_matrix = spones(one_hot_matrix);
end
