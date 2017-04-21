function [pcd_merged] = merge_scenes(frames, step, method)
    pcd_merged = zeros(0,3);
    if (strcat(method, 'method1'))
        R_cum = eye(3);
        t_cum = zeros(1, 3);
    end
    
    sampling_type = 'uniform';
    sampling_percentage = 0.05;

    for frame_id = 0:step:(frames - step)
        % base frame
        frame_str = sprintf('%010d', frame_id);
        pcd_base = strcat('data/', frame_str, '.pcd');
        jpg_base = strcat('data/', frame_str, '.jpg');
        mask_base = strcat('data/', frame_str, '_mask.jpg');
        depth_base = strcat('data/', frame_str, '_depth.png');
        
        % target frame
        frame_str2 = sprintf('%010d', frame_id + step);
        pcd_target = strcat('data/', frame_str2, '.pcd');
        jpg_target = strcat('data/', frame_str2, '.jpg');
        mask_target = strcat('data/', frame_str2, '_mask.jpg');
        depth_target = strcat('data/', frame_str2, '_depth.png');


        [pcd_base, ordered] = pcdFromDepth(depth_base);
        [pcd_target, ordered] = pcdFromDepth(depth_target);        
%         pcd_base = readPcd(pcd_base); pcd_base = pcd_base(:,1:3);
%         pcd_target = readPcd(pcd_target); pcd_target = pcd_target(:,1:3);

        % remove background
        [pcd_base, ~] = remove_background(pcd_base);
        [pcd_target, ~] = remove_background(pcd_target);

        fprintf('Merging frame %d\n', frame_id + step);
        
        if frame_id == 0 && strcmp(method, 'method2')
            pcd_merged = pcd_base;
            continue;
        end
        
        
        if (strcmp(method, 'method1'))
            [R, t, RMS]= iterative_closest_point(pcd_target, pcd_base, sampling_type, sampling_percentage);
            
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
            
            [R, t] = iterative_closest_point(pcd_target, pcd_merged, sampling_type, sampling_percentage);
            
            %print before
            figure(1);
            title(sprintf('Merging frame %d',  frame_id + step));
            subplot(1,2,1);
            scatter3(pcd_merged(:,1), pcd_merged(:,2), pcd_merged(:,3),'.');
            hold on
            scatter3(pcd_target(:,1), pcd_target(:,2), pcd_target(:,3),'.');
            hold off
            
            %transform the merged pointclouds
            pcd_merged = (pcd_merged - t) / R;
             
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