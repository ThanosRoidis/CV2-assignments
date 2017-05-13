function [ dense_blocks ] = get_dense_blocks( measurement_matrix, one_hot_matrix ) 

    [m,n] = size(measurement_matrix);
    
    if( nargin < 2)
        one_hot_matrix = measurement_matrix(1:2:end,:);  
        one_hot_matrix = spones(one_hot_matrix);
    end

    dense_blocks = {};
    
    %Create a dense block for each point 'i' in the measurement matrix (column)
    %with all the other points that are at least in all of the views that
    %'i' is in.
    for i=1:n

        common_views = one_hot_matrix(:,i);
        %'i' must be in at least 4 views
        if length(nonzeros(common_views)) > 3

            points = [i];

            for j = i:n
                if j ~= i 
                    other_views = one_hot_matrix(:,j);
                    if isequal(common_views, common_views & other_views)
                        points(end+1) = j;
                    end
                end
            end 
            
            %the dense block must have at least 10 points
            if length(points) >= 10
                common_views = find(common_views);

                common_views_xy = zeros(2*length(common_views),1);
                for k = 1:length(common_views)
                    common_views_xy(2*k - 1) = 2*common_views(k)-1;
                    common_views_xy(2*k) = 2*common_views(k);
                end

                dense_block = measurement_matrix(common_views_xy, points);
                dense_blocks{end + 1} = dense_block;
            end
        end

    end
end
