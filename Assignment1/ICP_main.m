source = load('source.mat');
source = source.source';
target = load('target.mat');
target = target.target';

[R,t,RMS] = iterative_closest_point(source, target, 'all', 0.5, [],[]);

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