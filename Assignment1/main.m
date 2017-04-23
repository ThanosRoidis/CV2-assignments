source = load('source.mat');
source = source.source';
target = load('target.mat');
target = target.target';

[R,t,RMS] = iterative_closest_point(source, target, 'all', 0.5);

estimated = source * R + t;

figure(1);

subplot(1,2,1);
scatter3(target(:,1), target(:,2), target(:,3));
hold on
scatter3(source(:,1), source(:,2), source(:,3));
hold off

%figure(2);
subplot(1,2,2);
scatter3(target(:,1), target(:,2), target(:,3));
hold on
scatter3(estimated(:,1), estimated(:,2), estimated(:,3));
hold off



% 
% %test merging scenes
%   point_cloud_merged = merge_scenes(99, 1, 'method2');
%  ind = randsample(size(point_cloud_merged,1), 40000);
% X = point_cloud_merged(ind,1);
% Y = point_cloud_merged(ind,2);
% Z = point_cloud_merged(ind,3);
% C = X;
% % X = point_cloud_merged(:,1);
% % Y = point_cloud_merged(:,2);
% % Z = point_cloud_merged(:,3);
% % C = X; 
% fscatter3(X,Y,Z,C);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');