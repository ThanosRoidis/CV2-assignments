% check if calling vl_sift gives the same results each time
% im1 = 'House/frame00000049.png';
% im1 = imread(im1);
% imshow(im1);
% if size(im1,3) == 3
%     im1 = rgb2gray(im1);
% end
% 
% Ia = single(im1);
% [fa, da] = vl_sift(Ia) ;
% [fb, db] = vl_sift(Ia) ;
% 
% disp(sum(fa - fb))
% disp(sum(da - db))
%%
% Create the matches for each image pair
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




%%
matches_pairs = load('matches_pairs.mat');
matches_pairs = matches_pairs.matches_pairs;
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
% pvm = importdata('PointviewMatrix.txt');
% [M,S] = factorize(pvm, true);
% figure(1);
% scatter3(S(1,:), S(2,:), S(3,:), '.');


%% test background removal
% 
% %image and mask
% I  = imread('House/frame00000001.png');
% level = graythresh(I);
% BW = imbinarize(I,level);
% I_bf = uint8(BW) .* I;
% % figure(1)
% % imshowpair(I,BW,'montage')
% 
% 
% %Descriptors with and without mask
% figure(2);
% 
% Ia = single(I); 
% [fa, ~] = vl_sift(Ia) ; 
% 
% Ib = single(I_bf); 
% [fb, ~] = vl_sift(Ib) ; 
% 
% imshowpair(I,I_bf,'montage')
% hold on
% scatter(fa(1,:), fa(2,:), '*')
% scatter(size(I,2) + fb(1,:),  fb(2,:), '*')
% title('SIFT with and SIFT without background');
% 
% 
% %Image with given descriptors and no background with descriptors
% figure(3);
% imshowpair(I,I_bf,'montage')
% hold on
% scatter(pvm(1,:), pvm(2,:), '*')
% scatter(size(I,2) + fb(1,:),  fb(2,:), '*')
% title('Given inliers and SIFT without background');
% 
% 
% 
% %Image with given descriptors and no background with descriptors
% figure(4);
% [~, inliers] = fundamental_RANSAC(matches_pairs{1}, 8, 1, 20, false);
% imshowpair(I,I,'montage')
% hold on
% scatter(pvm(1,:), pvm(2,:), '*')
% scatter(size(I,2) + inliers(:,1),  inliers(:,2), '*')
% title('Given inliers and our inliers WITH background');
% 
% 
% figure(5);
% 
% I2  = imread('House/frame00000002.png');
% level = graythresh(I2);
% BW = imbinarize(I2,level);
% I_bf2 = uint8(BW) .* I2;
%  
% [fa, da] = vl_sift(single(I_bf)) ; 
% [fb, db] = vl_sift(single(I_bf2)); 
% 
% [matches_indexes, scores] = vl_ubcmatch(da, db) ;
% matches = zeros(length(matches_indexes), 4);
% for i=1:length(matches_indexes)
%     matches(i,1:2) = fa(1:2,matches_indexes(1,i));
%     matches(i,3:4) = fb(1:2,matches_indexes(2,i));
% end
% [~, inliers] = fundamental_RANSAC(matches, 8, 0.1, 200, false);
% 
% imshowpair(I,I,'montage')
% hold on
% scatter(pvm(1,:), pvm(2,:), '*')
% scatter(size(I,2) + inliers(:,1),  inliers(:,2), '*')
% title('Given inliers and our inliers WITHOUT background');

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
%%
disp('Calculating measurement matrix...');
[measurement_matrix, one_hot_matrix] = chaining(matches_pairs);

disp('Calculating dense blocks...');
dense_blocks = get_dense_blocks( measurement_matrix, one_hot_matrix );

pcds = {};

disp('Factorizing dense blocks...');
for i=1:length(dense_blocks)
    [M,S] = factorize(dense_blocks{i}, false);
    pcds{end + 1} = S;
end



