num_of_buckets = 4;
percentage = 0.5;

%% create buckets
for frame_id = 0:1
    % get file path
    frame_str = sprintf('%010d', frame_id);
    normal = strcat('data/', frame_str, '_normal.pcd');
    
    % load normal
    normals = readPcd(normal);
    
    % remove nans
    normals(isnan(normals(:,1)),:)=[];
    
    angles_x = [];
    angles_y = [];
    
    for i=1 : size(normals,1)
        p = normals(i,:);
        
        % angle in rand w.r.t axis x
        x = atan2(sqrt(p(2)^2 + p(3)^2), p(1));
        % angle in rand w.r.t axis y
        y = atan2(sqrt(p(1)^2 + p(3)^2), p(2));
        
        angles_x = [angles_x ; x];
        angles_y = [angles_y ; y];
    end
    
    % create buckets w.r.t angles
    [bucket_x, edges_x] = discretize(angles_x, num_of_buckets);
    [bucket_y, edges_y] = discretize(angles_y, num_of_buckets);
    
    bucket = num_of_buckets * (bucket_x - 1) + bucket_y;
    
    %
    max_id = num_of_buckets * num_of_buckets;
    inv_indices = cell(1, max_id);
    
    for i=1 : max_id
        k = find(bucket == i);
        inv_indices{i} = k;
    end
    
end

%% uniformly sample from buckets

% initial number of points
num_of_points = size(normals, 1);
% keep #num_of_sample points
num_of_sample = ceil (num_of_points * percentage);
% keep #num_of_sample_per_bucket from each bucket
num_of_sample_per_bucket = ceil(num_of_sample / max_id);

indexes_ = [];
for i=1 : max_id
    if num_of_sample_per_bucket < size(inv_indices{i}, 1)
        indexes = randsample(inv_indices{i}, num_of_sample_per_bucket);
        indexes_ = [indexes_; indexes];
    else
        indexes_ = [indexes_; inv_indices{i}];
    end
end
%% visualize buckets
% scatter3(normals(:,1),normals(:,2),normals(:,3),ones(size(normals,1),1),bucket)