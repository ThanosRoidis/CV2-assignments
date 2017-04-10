
source = load('source.mat');
source = source.source';
target = load('target.mat');
target = target.target';

[R,t] = iterative_closest_point(source, target);

estimated = source * R + t;

source = source';
target = target';
estimated = estimated';

figure(1);
scatter3(target(1,:), target(2,:), target(3,:));
hold on
scatter3(source(1,:), source(2,:), source(3,:));
hold off

figure(2);
scatter3(target(1,:), target(2,:), target(3,:));
hold on
scatter3(estimated(1,:), estimated(2,:), estimated(3,:));
hold off



%{
%R_targ = rand(3,3);
R_targ =  [1 0 0; 0 0 -1; 0 1 0];
%t_targ = rand(1,3);
t_targ = [1 1 1];


a = rand(10,3);
b = a*R_targ + t_targ;

[R,t] = svd_rot(a,b);
c = a*R + t;

diagonal = ones(5,1);
diagonal = diag(diagonal);
diagonal(5,5) = 5;
%}