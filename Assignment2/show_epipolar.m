%Show epipolar for the first matches between frame 1 and 2
% matches_pairs = load('matches_pairs.mat');
% matches_pairs = matches_pairs.matches_pairs;

% Plot epipolar lines
[F, inliers] = fundamental_RANSAC( matches_pairs{1},9,1,20); 
% plot epipolars
left_image  = imread('House/frame00000001.png');
right_image = imread('House/frame00000002.png');
plot_epipolar(left_image, right_image, inliers, F, 20);

%check if FX = 0
sum_H = 0;
for i=1 : size(inliers, 1)
    H = [inliers(i,3),inliers(i,4),1] * F *[inliers(i,1);inliers(i,2);1];
    sum_H = sum_H + H; 
end

fprintf('Average value of xFx\'': %d\n', sum_H/size(inliers,1));

