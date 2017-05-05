function [] = plot_epipolar(left_image, right_image, inliers, F)
[m n] = size(left_image);

figure,imagesc(left_image);
figure,imagesc(right_image);

for i=1 : size(inliers, 1)
    % select random point
    left_x = inliers(i,1);
    left_y = inliers(i,2);
    
    figure(1);
    hold on;
    plot(left_x, left_y, 'r*');
    
    
    left_P = [left_x; left_y; 1];
    right_P = F*left_P;
    right_epipolar_x = 1:2*m;
    right_epipolar_y = (-right_P(3)-right_P(1)*right_epipolar_x)/right_P(2);
    
    
    figure(2);
    hold on;
    plot(right_epipolar_x, right_epipolar_y, 'r');
    plot(inliers(i,3), inliers(i,4), 'r*');
    
    [~, ~, FV] = svd(F);
    left_epipole = FV(:,3);
    left_epipole = left_epipole/left_epipole(3);
    
    left_epipolar_x = 1:2*m;
    left_epipolar_y = left_y + (left_epipolar_x-left_x)*(left_epipole(2)-left_y)/(left_epipole(1)-left_x);
    
    figure(1); hold on; plot(left_epipolar_x, left_epipolar_y, 'r');
    
    
end

end