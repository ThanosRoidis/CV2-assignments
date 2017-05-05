% step = 1;
% last_frame = 49;
% 
% frames = {};
% for frame_id = 1:step:last_frame
%     % base frame filenames
%     frame_str = sprintf('House/frame%08d.png', frame_id);
%     frames{frame_id} = imread(frame_str);
% end
% 
% matches_pairs = {};
% c = 1;
% for cur = 1:(length(frames) - 1)
%     next = cur + step;
%     matches = get_matches(frames{cur}, frames{next});
%     matches_pairs{cur} = matches;
% end
matches_pairs = load('matches_pairs.mat');
matches_pairs = matches_pairs.matches_pairs;

% matches = matches_pairs{1};
% 
% %pick n=8 random pairs
% n = 8;
% r = randsample(size(matches,1),n);
% sampled = matches(r,:);
% 
% %build A
% A = zeros(n, 9);
% A(:,1) = sampled(:,1) .* sampled(:,3);
% A(:,2) = sampled(:,1) .* sampled(:,4);
% A(:,3) = sampled(:,1);
% A(:,4) = sampled(:,2) .* sampled(:,3);
% A(:,5) = sampled(:,2) .* sampled(:,4);
% A(:,6) = sampled(:,2);
% A(:,7) = sampled(:,3);
% A(:,8) = sampled(:,4);
% A(:,9) = ones(n, 1);
% 
% [U, D, V] = svd(A);
% 
% F = V(:,end);
% F = reshape(F, [3,3]);
% 
% %Enforce Singularity on F
% [Uf, Df, Vf] = svd(F);
% Df(end) = 0;
% F = Uf * Df * Vf';
% 
% T = eye(3);
% 
% F = T * F * T';
% 
% 
% %Sampson Distance
% 
% 
% 
% 

[F, inliers] = fundamental_RANSAC( matches_pairs{1},8,100);

%% plot epipolars
left_image  = imread('House/frame00000001.png');
right_image = imread('House/frame00000002.png');

plot_epipolar(left_image, right_image, inliers, F);

%% check if FX = 0
% for i=1 : size(inliers, 1)
%     H = [inliers(i,3),inliers(i,4),1] * F *[inliers(i,1);inliers(i,2);1];
%     disp(H);
% end