% test background removal

I  = imread('House/frame00000001.png');
level = graythresh(I);
BW = imbinarize(I,level);
I_bf = uint8(BW) .* I;

I2  = imread('House/frame00000002.png');
level = graythresh(I2);
BW = imbinarize(I2,level);
I_bf2 = uint8(BW) .* I2;



pvm = importdata('PointviewMatrix.txt');
matches_pairs = load('matches_pairs.mat');
matches_pairs = matches_pairs.matches_pairs;

%% Image with given descriptors and no background with descriptors
figure(2);
imshowpair(I,I_bf,'montage')
hold on
scatter(pvm(1,:), pvm(2,:), '*')
scatter(size(I,2) + fb(1,:),  fb(2,:), '*')
title('Given inliers and SIFT without background');

%% Image with given descriptors and no background with descriptors
figure(3);
[~, inliers] = fundamental_RANSAC(matches_pairs{1}, 8, 1, 20, false);
imshowpair(I,I,'montage')
hold on
scatter(pvm(1,:), pvm(2,:), '*')
scatter(size(I,2) + inliers(:,1),  inliers(:,2), '*')
title('Given inliers and our inliers WITH background');

%% 
figure(4);
[fa, da] =  vl_sift(single(I_bf)) ; %vl_covdet(single(I_bf), 'method', 'MultiscaleHarris') ;
[fb, db] =  vl_sift(single(I_bf2)); %vl_covdet(single(I_bf2), 'method', 'MultiscaleHarris') ;

[matches_indexes, scores] = vl_ubcmatch(da, db) ;
matches = zeros(length(matches_indexes), 4);
for i=1:length(matches_indexes)
    matches(i,1:2) = fa(1:2,matches_indexes(1,i));
    matches(i,3:4) = fb(1:2,matches_indexes(2,i));
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
