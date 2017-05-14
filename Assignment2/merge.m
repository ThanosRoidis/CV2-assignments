%function [pcd_merged] = merge(pcds, block_points, block_views)

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
existing_pointsIDs = block_points{1};

for i= 2 : length(pcds)
    
    pcd_target = pcds{i};
    
    % find which overlapping points does the tagret contain
    [shared_points, target_pos, merged_pos] = intersect(block_points{i}, existing_pointsIDs);
    
    %
    pcd_target_existing = pcd_target(:, target_pos);
    pcd_merged_overlap = pcd_merged(:, merged_pos);
    
    new = setdiff(1:size(pcd_target,2),target_pos)';
    pcd_target_new = pcd_target(:,new);
    %disp(new);
    %disp(target_pos);
    
    %skip frames with a few overlapping points
%     if size(ids, 1) < 4
%         fprintf('------Skip------');
%         continue;
%     end
    % ICP only on the shared points
    [R, T, ~] = icp(pcd_target_existing, pcd_merged_overlap, 5);
     
    % transform the merged pointclouds
    pcd_merged = R * pcd_merged + repmat(T,1,length(pcd_merged));
    
    
    % add the new frame to the merged
    pcd_merged = cat(2, pcd_merged, pcd_target_new);
    
    new_pointIDs = block_points{i}(new);
    existing_pointsIDs = [existing_pointsIDs, new_pointIDs];
    %existing_points = unique(existing_points);
    disp(i);
end

%end