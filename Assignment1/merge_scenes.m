function [pcd_merged] = merge_scenes(last_frame, step, method)
% merge the scenes until the 'last_frame' frame, starting from frame 0, with a step of 'step'.
%
    %The normals of the current frame, the previous frame and all of the
    %frames
    normals_base = [];
    normals_target = [];
    normals_merged = [];
    
    %The pointcloud of all the previous frames
    pcd_merged = zeros(0,3);
    %The total R and t used in the first method only
    if (strcat(method, 'method1'))
        R_cum = eye(3);
        t_cum = zeros(1, 3);
    end
    
    sampling_method = 'uniform'; %'random', 'normal'
    sampling_percentage = 0.01;
    
    %weighting_method = 'normals';
    %rejection_method = 'worst_percent';

    %Loop over all of the images, starting from 0
    for frame_id = 0:step:(last_frame - step)
        
        % base frame filenames
        frame_str = sprintf('%010d', frame_id);
        pcd_base = strcat('data/', frame_str, '.pcd');
        %jpg_base = strcat('data/', frame_str, '.jpg'); mask_base = strcat('data/', frame_str, '_mask.jpg'); depth_base = strcat('data/', frame_str, '_depth.png');
        normals_base = strcat('data/', frame_str, '_normal.pcd');
        
        % target frame filenames
        frame_str2 = sprintf('%010d', frame_id + step);
        pcd_target = strcat('data/', frame_str2, '.pcd');
        %jpg_target = strcat('data/', frame_str2, '.jpg'); mask_target = strcat('data/', frame_str2, '_mask.jpg'); depth_target = strcat('data/', frame_str2, '_depth.png');
        normals_target = strcat('data/', frame_str2, '_normal.pcd');

       %read pointclouds
        pcd_base = readPcd(pcd_base); pcd_base = pcd_base(:,1:3);
        pcd_target = readPcd(pcd_target); pcd_target = pcd_target(:,1:3);

        % remove background
        [pcd_base, ids1] = remove_background(pcd_base);
        [pcd_target, id2] = remove_background(pcd_target);
        
        %read normals if needed
        if strcmp(sampling_method, 'normal') ||  strcmp(weighting_method, 'normals')
            normals_base = readPcd(normals_base);
            normals_base = normals_base(ids1, 1:3);
            normals_target = readPcd(normals_target);
            normals_target = normals_target(id2, 1:3);
        end
        
        fprintf('Merging frame %d\n', frame_id + step);
        
        %The first method
        if (strcmp(method, 'method1'))
            [R, t, RMS]= iterative_closest_point(pcd_target, pcd_base, sampling_method...
                , sampling_percentage, normals_target, normals_base);
            
            %skip frames with a high RMS
            if RMS(end) > 0.5
                fprintf('High RMS (%f), skipping frame!\n', RMS(end));
                continue;
            end
            %Accumulate R and t
            t_cum = t * R_cum + t_cum;
            R_cum = R * R_cum;
            
            %Transform the pointcloud of the current frame
            transf_pcd = pcd_target * R_cum + t_cum;
            
            %append it to the total point cloud
            pcd_merged = cat(1, pcd_merged, transf_pcd);
            
            %Plot the pointclouds before and after
            figure(1);
            title(sprintf('Merging frame %d',  frame_id + step));
            subplot(1,2,1);
            scatter3(pcd_base(:,1), pcd_base(:,2), pcd_base(:,3),'.');
            hold on
            scatter3(pcd_target(:,1), pcd_target(:,2), pcd_target(:,3),'.');
            hold off
            
            subplot(1,2,2);
            scatter3(pcd_merged(:,1), pcd_merged(:,2), pcd_merged(:,3),'.');
            hold on
            scatter3(transf_pcd(:,1), transf_pcd(:,2), transf_pcd(:,3),'.');
            hold off
            drawnow;

        elseif(strcmp(method, 'method2'))
            %On the first frame, initialize it as the total point clouds
            if frame_id == 0 
                pcd_merged = pcd_base;
                normals_merged = normals_base;
            end
            
            [R, t] = iterative_closest_point(pcd_target, pcd_merged, sampling_method, sampling_percentage, normals_target, normals_merged);
            
            %plot before
            figure(1);
            title(sprintf('Merging frame %d',  frame_id + step));
            subplot(1,2,1);
            scatter3(pcd_merged(:,1), pcd_merged(:,2), pcd_merged(:,3),'.');
            hold on
            scatter3(pcd_target(:,1), pcd_target(:,2), pcd_target(:,3),'.');
            hold off
            
            %transform the merged pointclouds
            pcd_merged = (pcd_merged - t) / R;
            if ~isempty(normals_merged)
                normals_merged = normals_merged / R;
            end
            
            %plot after
            subplot(1,2,2); 
            scatter3(pcd_merged(:,1), pcd_merged(:,2), pcd_merged(:,3),'.');
            hold on
            scatter3(pcd_target(:,1), pcd_target(:,2), pcd_target(:,3),'.');
            hold off
            drawnow;
            
            %add the new frame to the merged
            pcd_merged = cat(1, pcd_merged, pcd_target);
            if ~isempty(normals_merged)
                normals_merged = cat(1, normals_merged, normals_target);
            end
            
        end
    end

    disp('Merging Finished!!!');
 end


function  [cloud, ordered] = pcdFromDepth(file_path)

    fx = 526.37013657; %focal length in width
    fy = 526.37013657; %focal length in height
    cx = 313.68782938; %principal point in width
    cy = 259.01834898; %principal point in height

    %file_path = 'depth.png'; %depth image path
    depth = imread(file_path); %load depth image
    depth = double(depth) * 0.001; %scale depth image from mm to meter.
    [cloud, ordered]= depth2cloud(depth, fx, fy,cx,cy); % convert from the depth image to cloud

end

function [pointCloud_filtered, indexes] = remove_background(point_cloud)
%remove all of the points that are in the background (2 meters away)
    threshold = 2;

    indexes = find(point_cloud(:,3) < threshold);
    pointCloud_filtered = point_cloud(indexes,:);
end