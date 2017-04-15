%% iterative closest point
function [R,t, RMS] = iterative_closest_point(A1, A2, sampling_type, sampling_percentage)
    
    if true
        verbose = true;
    end

    %% Initialize R = I, t = 0
    R = eye(3);
    t = zeros(1, 3);

    A1_init = A1;
    A2_init = A2;
    %% subsample source and target
    A1 = select_points(A1, sampling_percentage, sampling_type);
    A2 = select_points(A2, sampling_percentage, sampling_type);

    A1_transf = A1 * R + t;

    %% Find the closest points for each point in the base point set (A1) from the target point set (A2) using brute-force approach.
    for i = 1 : 40

        % for random sampling, a different sample of points is taken at each iteration
        if strcmp(sampling_type, 'random')
            A1 = select_points(A1_init, sampling_percentage, sampling_type);
            A1_transf = A1 * R + t;
            A2 = select_points(A2_init, sampling_percentage, sampling_type);
        end
        
        %Find psi(A) using pairwise distance
        pairwise_dist = pdist2(A1_transf, A2);
        [~, index] = min(pairwise_dist, [], 2);
        psi_A1 = A2(index, :);
        
        %Find rotation
        [R,t] = svd_rot(A1, psi_A1);

        %Update Points
        A1_transf = A1 * R + t;

        %Calculate Root Mean Square
        dist = RootMeanSquare(A1_transf, psi_A1);
        if i == 1
            RMS = dist;
        else
            RMS = cat(1,RMS, dist);
        end
        
        
        if verbose
            fprintf('Iteration %d RMS: %f\n', i, dist);
        end
    end
end

%% Root Mean Square
function dist = RootMeanSquare(A1_transf, psi_A1)
    dist = sum(sum(((A1_transf-psi_A1).^2), 2));
    %dist = sum(norm(A1_transf-psi_A1) ^ 2);
end

    %% Sampling
function [sampled_points] = select_points(points, percentage, type)

    % initial number of points
    num_of_points = size(points, 1);

    % keep #num_of_sample points
    num_of_sample = ceil (num_of_points * percentage);

    if (strcmp(type, 'all'))
        sampled_points = points;

    elseif (strcmp(type, 'uniform'))    
        indexes = int64(linspace(1, num_of_points, num_of_sample));
        sampled_points = points(indexes, :);

    elseif (strcmp(type, 'random'))
            indexes = randsample(num_of_points, num_of_sample);
            sampled_points = points(indexes, :);

    end

end