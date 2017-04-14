function [] = merge_scenes(frames, step, method)

point_could_merged = zeros();

for frame_id=0 : step : (frames - step)
    
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
    
    
    [point_cloud_base, ordered] = demo(depth_base);
    [point_cloud_target, ordered] = demo(depth_target);
    
    % remove background
    [point_cloud_base, ~] = remove_background(point_cloud_base);
    [point_cloud_target, ~] = remove_background(point_cloud_target);
    
    
    if (step == 1)
        [R, t] = iterative_closest_point(point_could_merged(1:index, :), point_cloud_target, sampling_type, sampling_percentage);
        
        
    else
        [R, t]= iterative_closest_point(point_cloud_base, point_cloud_target, sampling_type, sampling_percentage);
        
    end
end

end


function  [cloud, ordered] = demo(file_path)

fx = 526.37013657; %focal length in width
fy = 526.37013657; %focal length in height
cx = 313.68782938; %principal point in width
cy = 259.01834898; %principal point in height

file_path = 'depth.png'; %depth image path
depth = imread(file_path); %load depth image
depth = double(depth) * 0.001; %scale depth image from mm to meter.
[cloud, ordered]= depth2cloud(depth, fx, fy,cx,cy); % convert from the depth image to cloud

end

function [pointCloud_filtered, indexes] = remove_background(point_cloud)
threshold = 10;

indexes = point_cloud(:,3) < threshold;
pointCloud_filtered = point_cloud(indexes,:);

end