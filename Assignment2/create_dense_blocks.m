
measurement_matrix = load('measurement_matrix.mat');
measurement_matrix = measurement_matrix.measurement_matrix;


[m,n] = size(measurement_matrix);
one_shot = measurement_matrix(1:2:end,:);  % odd matrix
one_shot = spones(one_shot);

one_shot2 = zeros(m/2,n);
for i = 1:2:m
    for j = 1:n
        if measurement_matrix(i,j) > 0
            one_shot2(floor(i/2) + 1, j) = 1;
        end
    end 
end

% disp(sum(sum(one_shot2 - one_shot,1)));


dense_blocks = {};
for i=1:n
 
    common_views = one_shot(:,i);
    if length(nonzeros(common_views)) > 3

        points = [i];

        for j = i:n
            if j ~= i 
                other_views = one_shot(:,j);
                if isequal(common_views, common_views & other_views)
                    points(end+1) = j;
                end
            end
        end 
        
        if length(points) > 10
            common_views = find(common_views);

            common_views_xy = zeros(2*length(common_views),1);
            for k = 1:length(common_views)
                common_views_xy(2*k - 1) = 2*common_views(k)-1;
                common_views_xy(2*k) = 2*common_views(k);
            end

            dense_block = measurement_matrix(common_views_xy, points);
            dense_blocks{end + 1} = dense_block;
        end
    end

end
