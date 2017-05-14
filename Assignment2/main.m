%%
matches_pairs = load('matches_pairs.mat');
matches_pairs = matches_pairs.matches_pairs;

%% Create the matches for each image pair
% step = 1;
% last_frame = 49;
% remove_bg = false;
% frames = {};
% for frame_id = 1:step:last_frame
%     frame_str = sprintf('House/frame%08d.png', frame_id);
%     frames{frame_id} = imread(frame_str);
%     if remove_bg
%         level = graythresh(frames{frame_id});
%         BW = imbinarize(frames{frame_id},level);
%         frames{frame_id} = uint8(BW) .* frames{frame_id};
%     end
% end
% 
% matches_pairs = {};
% c = 1;
% for cur = 1:(length(frames) - 1)
%     next = cur + step;
%     matches = get_matches(frames{cur}, frames{next});
%     matches_pairs{cur} = matches;
% end

%% Plot epipolar lines
% [F, inliers] = fundamental_RANSAC( matches_pairs{1},9,1,20);
% % 
% % %% plot epipolars
% left_image  = imread('House/frame00000001.png');
% right_image = imread('House/frame00000002.png');
% plot_epipolar(left_image, right_image, inliers, F, 20);

% check if FX = 0
% for i=1 : size(inliers, 1)
%     H = [inliers(i,3),inliers(i,4),1] * F *[inliers(i,1);inliers(i,2);1];
%     disp(H);
% end

%% test factorize
%pvm = importdata('PointviewMatrix.txt');
% [M,S] = factorize(pvm, true);
% figure(1);
% scatter3(S(1,:), S(2,:), S(3,:), '.');



%% Whole pipeline
disp('Calculating measurement matrix...');
[measurement_matrix, one_hot_matrix] = chaining(matches_pairs);
fprintf(' Size: %d x %d (%d points in %d views)\n', [size(measurement_matrix), size(one_hot_matrix')]);
% 
disp('Calculating dense blocks of the measurement matrix...');
[dense_blocks, block_points, block_views] = get_dense_blocks( measurement_matrix, one_hot_matrix );
fprintf(' %d dense blocks\n', length(dense_blocks));

pcds = {};

disp('Factorizing dense blocks...');
for i=1:length(dense_blocks)
    [M,S] = factorize(dense_blocks{i}, false);
    pcds{end + 1} = S;
end


cur = pcds{18};
ind = find(abs(cur(1,:) < 5)); %remove noise
cur = cur(:,ind);
scatter3(cur(1,:), cur(2,:), cur(3,:),'.');

