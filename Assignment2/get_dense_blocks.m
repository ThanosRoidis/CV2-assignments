function [ dense_blocks, block_points, block_views] = get_dense_blocks( measurement_matrix, one_hot_matrix ) 
%GET_DENSE_BLOCKS It receives the sparse measurement matrix (and the optionary
%one_hot_matrix), and returns a cell array in which each element is a dense
%block. A dense block is created for each point 'i' in the measurement
%matrix (column) that is in at least 3 other views, with all the other points that are at least in all of the
%views that 'i' is in. It also returns the IDs of the points and the
%views of each dense block

    [m,n] = size(measurement_matrix);
    
    if( nargin < 2)
        one_hot_matrix = measurement_matrix(1:2:end,:);  
        one_hot_matrix = spones(one_hot_matrix);
    end
    
    dense_blocks = {};
    block_points = {};
    block_views = {};
    
    %Create a dense block for each point 'i' in the measurement matrix (column)
    %with all the other points that are at least in all of the views that
    %'i' is in.
    for i=1:n
    
        %the views of point/column 'i'
        common_views = one_hot_matrix(:,i);
        
        %if there is already another block with the same views, then skip
        same_views = false;
        for k = 1:length(block_views)
            if isequal(block_views{k}, find(common_views)) 
                same_views = true;
                break;
            end
        end
        if same_views 
            continue;
        end
        
        %'i' must be in at least 4 views
        if length(nonzeros(common_views)) > 3

            %find all the points that are at least in the same views as 'i'
            points = [i];
            for j = 1:n
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
                
                block_points{end + 1} = points;
                block_views{end + 1} = common_views;
            end
        end
    end
   
end
