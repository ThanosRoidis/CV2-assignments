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
pcd_merged = zeros(0,3);

%
overlapping_points = [];

for i=1 : size(pcds, 2)
    
    %On the first frame, initialize it as the total point clouds
    if i == 1
        pcd_merged = pcds{i};
        overlapping_points = [overlapping_points; block_points{i}];
    end
    pcd_target = pcds{i+1};
    
    % find which overlapping points does the tagret contain
    [shared_points, ids] = intersect(block_points{i+1}, overlapping_points);
    
    %
    pcd_target_shared = pcd_target(:, ids');
    
    %skip frames with a few overlapping points
%     if size(ids, 1) < 4
%         fprintf('------Skip------');
%         continue;
%     end
    
    % ICP only on the shared points
    [R, T, ~] = icp(pcd_target_shared, pcd_merged, 20);
    
    % transform the merged pointclouds
    pcd_merged = (pcd_merged - T) \ R;
    
    % add the new frame to the merged
    pcd_merged = cat(2, pcd_merged', pcd_target);
    disp(i);
end

%end