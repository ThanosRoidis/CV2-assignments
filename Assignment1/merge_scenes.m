function [pcd_merged] = merge_scenes(frames, step, method)
    normals_base = [];
    normals_target = [];
    pcd_merged = zeros(0,3);
    if (strcat(method, 'method1'))
        R_cum = eye(3);
        t_cum = zeros(1, 3);
    end
    
    sampling_method = 'uniform';
    weighting_method = 'normals';
    rejection_method = 'worst_percent';
    sampling_percentage = 0.1;

    %Loop over all of the images, starting from 0
    for frame_id = 0:step:(frames - step)
        % base frame
        frame_str = sprintf('%010d', frame_id);
        pcd_base = strcat('data/', frame_str, '.pcd');
        %jpg_base = strcat('data/', frame_str, '.jpg'); mask_base = strcat('data/', frame_str, '_mask.jpg'); depth_base = strcat('data/', frame_str, '_depth.png');
        normals_base = strcat('data/', frame_str, '_normal.pcd');
        
        % target frame
        frame_str2 = sprintf('%010d', frame_id + step);
        pcd_target = strcat('data/', frame_str2, '.pcd');
        %jpg_target = strcat('data/', frame_str2, '.jpg'); mask_target = strcat('data/', frame_str2, '_mask.jpg'); depth_target = strcat('data/', frame_str2, '_depth.png');
        normals_target = strcat('data/', frame_str2, '_normal.pcd');


%       [pcd_base, ordered] = pcdFromDepth(depth_base);
%       [pcd_target, ordered] = pcdFromDepth(depth_target);        
        pcd_base = readPcd(pcd_base); pcd_base = pcd_base(:,1:3);
        pcd_target = readPcd(pcd_target); pcd_target = pcd_target(:,1:3);

        % remove background
        [pcd_base, ids1] = remove_background(pcd_base);
        [pcd_target, id2] = remove_background(pcd_target);
        
        %read normals
        if strcmp(sampling_method, 'normal') ||  strcmp(weighting_method, 'normals')
            normals_base = readPcd(normals_base);
            normals_base = normals_base(ids1, 1:3);
            normals_target = readPcd(normals_target);
            normals_target = normals_target(id2, 1:3);
        end
        
        fprintf('Merging frame %d\n', frame_id + step);
        
        
        if (strcmp(method, 'method1'))
            [R, t, RMS]= iterative_closest_point(pcd_target, pcd_base, sampling_method...
                , sampling_percentage, normals_target, normals_base);
            
            if RMS(end) > 0.5
                fprintf('High RMS (%f), skipping frame!\n', RMS(end));
                continue;
            end
            
            t_cum = t * R_cum + t_cum;
            R_cum = R * R_cum;
            
            transf_pcd = pcd_target * R_cum + t_cum;
            
            pcd_merged = cat(1, pcd_merged, transf_pcd);
            
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
            if frame_id == 0 
                pcd_merged = pcd_base;
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
             
            %plot after
            subplot(1,2,2); 
            scatter3(pcd_merged(:,1), pcd_merged(:,2), pcd_merged(:,3),'.');
            hold on
            scatter3(pcd_target(:,1), pcd_target(:,2), pcd_target(:,3),'.');
            hold off
            drawnow;
            
            %add the new frame to the merged
            pcd_merged = cat(1, pcd_merged, pcd_target);
            
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
    threshold = 2;

    indexes = find(point_cloud(:,3) < threshold);
    pointCloud_filtered = point_cloud(indexes,:);
end