%Run the merge scenes algorithm on all of the images with a step size of 1, using the first
%method
%DEFINE THE WEIGHTING AND REJECTION OF PAIRS INSIDE THE
%iterative_closest_point METHOD AND SAMPLING IN THE merge_scenes FUNCTION
point_cloud_merged = merge_scenes(99, 1, 'method1');

%Plot results
ind = randsample(size(point_cloud_merged,1), 40000);
X = point_cloud_merged(ind,1);
Y = point_cloud_merged(ind,2);
Z = point_cloud_merged(ind,3);
C = X;
figure(2);
fscatter3(X,Y,Z,C);
xlabel('X');
ylabel('Y');
zlabel('Z');