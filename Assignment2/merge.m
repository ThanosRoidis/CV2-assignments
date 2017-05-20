function [pcd_merged] = merge(pcds, block_points, block_views, method)
% Merges the given pcds using one of the 2 specified methods:
%   method 1 ->  ICP on all points
%   method 2 ->  ICP only on the shared points
%   block_points are the point IDs of each pcd
%   block_views are the views of each pcd

    if method == 1
        % find most pointed
        temp = [];
        for i=1: size(block_views,2)
            temp = [temp, length(block_views{i})];
        end

        [~, id] = max(temp);

        % swap
        temp_pcd = pcds{1};
        pcds{1} = pcds{id};
        pcds{id} = temp_pcd;

        temp_block_points = block_points{1};
        block_points{1} = block_points{id};
        block_points{id} = temp_block_points;

        temp_block_views = block_views{1};
        block_views{1} = block_views{id};
        block_views{id} = temp_block_views;
        %


        %The pointcloud of all the previous frames
        pcd_merged = pcds{1};
        pcd_merged = pcd_merged';


        for i= 2 : length(pcds)

            pcd_target = pcds{i}';

            % ICP on all points
            %On the first frame, initialize it as the total point clouds
            [R, t] = iterative_closest_point(pcd_target, pcd_merged);

            %transform the merged pointclouds
            pcd_merged = (pcd_merged - t) / R;

            %add the new frame to the merged
            pcd_merged = cat(1, pcd_merged, pcd_target);
        end


    elseif method == 2

        % find most pointed
        temp = [];

        for i=1: size(block_views,2)

            temp = [temp, length(block_views{i})];

        end
        [~, id] = max(temp);

        % swap
        temp_pcd = pcds{1};
        pcds{1} = pcds{id};
        pcds{id} = temp_pcd;

        temp_block_points = block_points{1};
        block_points{1} = block_points{id};
        block_points{id} = temp_block_points;

        temp_block_views = block_views{1};
        block_views{1} = block_views{id};
        block_views{id} = temp_block_views;
        %

        %The pointcloud of all the previous frames
        pcd_merged = pcds{1};
        pcd_merged = pcd_merged';
        existing_pointsIDs = block_points{1};

        for i= 2 : length(pcds)
            pcd_target = pcds{i}';

            % find which overlapping points does the tagret contain
            [shared_points, target_pos, merged_pos] = intersect(block_points{i}, existing_pointsIDs);

            %
            pcd_target_existing = pcd_target(target_pos,:);
            pcd_merged_overlap = pcd_merged(merged_pos,:);
            new = setdiff(1:size(pcd_target,1),target_pos)';
            pcd_target_new = pcd_target(new,:);

            % ICP only on the shared points
            %On the first frame, initialize it as the total point clouds
            [R, t] = iterative_closest_point(pcd_target_existing, pcd_merged_overlap);

            %transform the merged pointclouds
            pcd_merged = (pcd_merged - t) / R;

            %add the new frame to the merged
            pcd_merged = cat(1, pcd_merged, pcd_target_new);

            new_pointIDs = block_points{i}(new);
            existing_pointsIDs = [existing_pointsIDs, new_pointIDs];

        end

    end

end