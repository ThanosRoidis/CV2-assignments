% This script simply tests the background removal on finding matches/inliers

I  = imread('House/frame00000001.png');
level = graythresh(I);
BW = imbinarize(I,level);
I_bf = uint8(BW) .* I;

I2  = imread('House/frame00000002.png');
level = graythresh(I2);
BW = imbinarize(I2,level);
I_bf2 = uint8(BW) .* I2;

[f1, d1] =  vl_sift(single(I)) ; %vl_covdet(single(I_bf), 'method', 'MultiscaleHarris') ;
[f2, d2] =  vl_sift(single(I2)); %vl_covdet(single(I_bf2), 'method', 'MultiscaleHarris') ;

[f1_no, d1_no] =  vl_sift(single(I_bf)) ; %vl_covdet(single(I_bf), 'method', 'MultiscaleHarris') ;
[f2_no, d2_no] =  vl_sift(single(I_bf2)); %vl_covdet(single(I_bf2), 'method', 'MultiscaleHarris') ;


pvm = importdata('PointviewMatrix.txt');
matches_pairs = load('matches_pairs.mat');
matches_pairs = matches_pairs.matches_pairs;

%% Image with given descriptors and no background with descriptors
figure(1);
imshowpair(I,I_bf,'montage')
hold on
 
scatter(f1(1,:), f1(2,:), '*')
scatter(size(I,2) + f1_no(1,:),  f1_no(2,:), '*')
title('Keypoints with and without background');



%% Image with given descriptors and no background with descriptors
figure(2);
imshowpair(I,I_bf,'montage')
hold on
scatter(pvm(1,:), pvm(2,:), '*')
scatter(size(I,2) + f1_no(1,:),  f1_no(2,:), '*')
title('Given inliers and SIFT without background');

%%  Given inliers and our inliers with background
figure(3);
[~, inliers] = fundamental_RANSAC(matches_pairs{1}, 8, 1, 20, false);
imshowpair(I,I,'montage')
hold on
scatter(pvm(1,:), pvm(2,:), '*')
scatter(size(I,2) + inliers(:,1),  inliers(:,2), '*')
title('Given inliers and our inliers WITH background');

%% Given inliers and our inliers without background
figure(4);

[matches_indexes, scores] = vl_ubcmatch(d1_no, d2_no) ;
matches = zeros(length(matches_indexes), 4);
for i=1:length(matches_indexes)
    matches(i,1:2) = f1_no(1:2,matches_indexes(1,i));
    matches(i,3:4) = f2_no(1:2,matches_indexes(2,i));
end
[~, inliers] = fundamental_RANSAC(matches, 8, 0.1, 100, false);

imshowpair(I,I,'montage')
hold on
scatter(pvm(1,:), pvm(2,:), '*')
scatter(size(I,2) + inliers(:,1),  inliers(:,2), '*')
title('Given inliers and our inliers WITHOUT background');



%%  
% figure(5);
% Im = medfilt2(im2double(I), [16 16]);
% I_back = im2double(I) ./ (Im + 1); 
% I_no_back = im2double(I) - im2double(I_back);
% 
% Ia = single(uint8(255*I_no_back)); 
% [fa, ~] = vl_sift(Ia) ; 
% imshow(I_no_back);
% hold on;
% scatter(fa(1,:), fa(2,:), '*')
