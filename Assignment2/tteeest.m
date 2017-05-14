

cur = pcd_merged;
ind = find(abs(cur(3,:) < 5)); %remove noise
cur = cur(:,ind);
scatter3(cur(1,:), cur(2,:), cur(3,:),'.');
box on
ax = gca;
ax.BoxStyle = 'full';

title('My title')
xlabel('X')
ylabel('Y')
zlabel('Z')