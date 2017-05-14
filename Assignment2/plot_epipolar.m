function [] = plot_epipolar(left_image, right_image, inliers, F, max_plot_points)
%PLOT_EPIPOLAR Plots the two images (left and right) and the epipolars of
%the points of the left image on the right image, and the epipolars on the
%left image.


    [m n] = size(left_image);

    %draw the two images
    figure,imagesc(left_image);
    figure,imagesc(right_image);


    if max_plot_points < size(inliers,1)
        plot_points = randsample(size(inliers,1),max_plot_points)';
    else
        plot_points = 1:size(inliers, 1);
    end
        
     for i = plot_points 
        % The current inlier to plot
        left_x = inliers(i,1);
        left_y = inliers(i,2);

        %plot the point on the left image
        figure(1);
        hold on;
        plot(left_x, left_y, 'r*');

        %Find the epipolar line on the right image
        left_P = [left_x; left_y; 1];
        right_P = F*left_P;
        right_epipolar_x = 1:2*m;
        right_epipolar_y = (-right_P(3)-right_P(1)*right_epipolar_x)/right_P(2);

        %Draw the epipolar and the matching points on the right image
        figure(2);
        hold on;
        plot(right_epipolar_x, right_epipolar_y, 'r');
        plot(inliers(i,3), inliers(i,4), 'r*');
        
        

        %Find the epipolar line on the left image (other way around)
        [~, ~, FV] = svd(F);
        left_epipole = FV(:,3);
        left_epipole = left_epipole/left_epipole(3);

        left_epipolar_x = 1:2*m;
        left_epipolar_y = left_y + (left_epipolar_x-left_x)*(left_epipole(2)-left_y)/(left_epipole(1)-left_x);

        %plot on the left image
        figure(1); hold on; plot(left_epipolar_x, left_epipolar_y, 'r');

    end

end