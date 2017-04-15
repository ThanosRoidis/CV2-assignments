
% source = load('source.mat');
% source = source.source';
% target = load('target.mat');
% target = target.target';
% 
% [R,t] = iterative_closest_point(source, target, 'random', 0.5);
% 
% estimated = source * R + t;
% 
% figure(1);
% 
% subplot(1,2,1);
% scatter3(target(:,1), target(:,2), target(:,3),'.');
% hold on
% scatter3(source(:,1), source(:,2), source(:,3),'.');
% hold off
% 
% %figure(2);
% subplot(1,2,2);
% scatter3(target(:,1), target(:,2), target(:,3),'.');
% hold on
% scatter3(estimated(:,1), estimated(:,2), estimated(:,3),'.');
% hold off

% %Test svd_dot 
% %R_targ = rand(3,3);
% R_targ =  [1 0 0; 0 0 -1; 0 1 0];
% %t_targ = rand(1,3);
% t_targ = [1 1 1];
% 
% a = rand(10,3);
% b = a*R_targ + t_targ;
% 
% [R,t] = svd_rot(a,b);
% c = a*R + t;
% 
% diagonal = ones(5,1);
% diagonal = diag(diagonal);
% diagonal(5,5) = 5;



% %test merging scenes
point_cloud_merged2 = merge_scenes(99, 1, 'method1');
% ind = randsample(size(point_could_merged2,1), 1000000);
% X = point_could_merged2(ind,1);
% Y = point_could_merged2(ind,2);
% Z = point_could_merged2(ind,3);
% C = X;
X = point_cloud_merged2(:,1);
Y = point_cloud_merged2(:,2);
Z = point_cloud_merged2(:,3);
C = X;%ones(size(X,1),1);
fscatter3(X,Y,Z,C);
xlabel('X');
ylabel('Y');
zlabel('Z');