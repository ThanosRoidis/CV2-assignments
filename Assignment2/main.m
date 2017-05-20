%% Create the matches for each image pair

last_frame = 49;
step = 1; %step = 1 uses all images, step = 2 skips every other image, and so on
remove_bg = false; %Remove background from the raw 2d images or not
frames = {};
%Read images/frames
for frame_id = 1:step:last_frame
    frame_str = sprintf('House/frame%08d.png', frame_id);
    frames{end + 1} = imread(frame_str);
    if remove_bg
        level = graythresh(frames{end});
        BW = imbinarize(frames{end},level);
        frames{end} = uint8(BW) .* frames{end};
    end
end

%Create matches using SIFT descriptors from vl_feat library
matches_pairs = {}; 
for cur = 1:(length(frames) - 1)
    disp(cur)
    next = cur + 1;
    matches = get_matches(frames{cur}, frames{next});
    matches_pairs{cur} = matches;
end

%save matches to a file
if remove_bg
    save('matches_pairs_no_bg.mat', 'matches_pairs');
else
    save('matches_pairs.mat', 'matches_pairs');
end


%% Create measurement matrix

% load matching pairs (not needed if calculated previously)
remove_bg = false;
if remove_bg
    matches_pairs = load('matches_pairs_no_bg.mat');
    matches_pairs = matches_pairs.matches_pairs;
else
    matches_pairs = load('matches_pairs.mat');
    matches_pairs = matches_pairs.matches_pairs;
end

disp('Calculating measurement matrix...');
[measurement_matrix, one_hot_matrix] = chaining(matches_pairs(1:end));
fprintf(' Size: %d x %d (%d points in %d views)\n', [size(measurement_matrix), size(one_hot_matrix')]);

%% Calculate dense blocks out of the measurement matrix 
disp('Calculating dense blocks of the measurement matrix...');
[dense_blocks, block_points, block_views] = get_dense_blocks( measurement_matrix, one_hot_matrix );
fprintf(' %d dense blocks\n', length(dense_blocks));

%% Factorize the dense blocks
disp('Factorizing dense blocks...');
pcds = {};
for i=1:length(dense_blocks)
    [M,S] = factorize(dense_blocks{i}, true);
    pcds{end + 1} = S;
end

%% Plot one dense block (first 18 views or the biggest dense block)

pcd_to_plot = -1;

% Find pcd with first 18 views
plot_18_views_flag = true;
if plot_18_views_flag
    for i=1:length(block_views) 
        if isequal(block_views{i}', 1:18)
            pcd_to_plot = i;
            break;
        end
    end
end

% If the first 18 views were not found or if we want to plot the pcd whose dense block has the most number of elements
if pcd_to_plot == -1 
    dense_sizes = []
    for i=1:length(block_views)
        dense_sizes(end+1) = numel(dense_blocks{i});
    end
    [~, max_block] = max(dense_sizes);
    disp(max_block);
    pcd_to_plot = max_block;
end

%Plot the pcd
cur = pcds{pcd_to_plot};
mean_z = mean(cur(3,:));
std_z = std(cur(3,:));
ind = find(abs(cur(3,:) - mean_z) < 2*std_z);  
cur = cur(:,ind);
subplot(1,2,1);
scatter3(cur(1,:), cur(2,:), cur(3,:),'.');
box on
ax = gca;
ax.BoxStyle = 'full';
title(sprintf('Views: %d - %d, number of points: %d', block_views{pcd_to_plot}(1),block_views{pcd_to_plot}(end), length(block_points{pcd_to_plot})));
xlabel('X')
ylabel('Y')
zlabel('Z')


%% Factorize and plot the given pointview matrix
pvm = importdata('PointviewMatrix.txt');
[M,S] = factorize(pvm, true);
subplot(1,2,2);
scatter3(S(1,:), S(2,:), -S(3,:), '.');
box on
ax = gca;
ax.BoxStyle = 'full';
title('From given pointview matrix');
xlabel('X')
ylabel('Y')
zlabel('Z') 

%% Merge
% disp('Merging PCDs using ICP...');
% pcd_merged =  merge(pcds, block_points, block_views, 2)';
% mean_z = mean(pcd_merged(3,:));
% std_z = std(pcd_merged(3,:));
% ind = find(abs(pcd_merged(3,:) - mean_z) < 2*std_z);
% pcd_merged = pcd_merged(:, ind);
% figure(3);
% scatter3(pcd_merged(1,:), pcd_merged(2,:), pcd_merged(3,:), '.');
% box on
% ax = gca;
% ax.BoxStyle = 'full';
% title('Merged pointcloud');
% xlabel('X');
% ylabel('Y');
% zlabel('Z'); 


