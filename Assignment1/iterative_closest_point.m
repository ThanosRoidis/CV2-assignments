%% iterative closest point
function [R,t, RMS,A1] = iterative_closest_point(A1, A2, sampling_type, sampling_percentage, frame_id1, frame_id2, normals1, normals2)
    firs=0;
    tol = 1e-6;
    max_iter = 100;

    if true
        verbose = true;
    end

    %% Initialize R = I, t = 0
    R = eye(3);
    t = zeros(1, 3);

    A1_init = A1;
    A2_init = A2;
    %% subsample source and target
%     A1 = select_points(A1, sampling_percentage, sampling_type, normals1, frame_id1);
%     A2 = select_points(A2, sampling_percentage, sampling_type, normals2, frame_id2);
    A1 = select_points(A1, sampling_percentage, sampling_type);
   A2 = select_points(A2, sampling_percentage, sampling_type);
    A1_transf = A1 * R + t;
    
    kd_mdl = KDTreeSearcher(A2);

    %% Find the closest points for each point in the base point set (A1) from the target point set (A2) using brute-force approach.
    tic;
    for i = 0 : max_iter
        
        % for random sampling, a different sample of points is taken at each iteration
        if strcmp(sampling_type, 'random')
            A1 = select_points(A1_init, sampling_percentage, sampling_type);
             A2 = select_points(A2_init, sampling_percentage, sampling_type);
            A1_transf = A1 * R + t;
        end
        
        %match points
        [matches, distances] =  match_pairs(A1_transf, A2, kd_mdl);
        psi_A1 = A2(matches, :);
        
        %reject pairs;
%       accepted_pairs = reject_pairs(distances)
%       A1 = A1(accepted_pairs,:);
%       psi_A1 = psi_A1(accepted_pairs,:);
%       distances = distances(accepted_pairs);
        
        %define weights
%       max_dist = max(distances);
%       distances = 1 - distances/max_dist;
        
        %Find rotation
        [R,t] = svd_rot(A1, psi_A1);

        %Update Points
        A1_transf = A1 * R + t;

        %Calculate Root Mean Square
        dist = RootMeanSquare(A1_transf, psi_A1);
        if i == 0
            RMS = dist;
        else
            RMS = cat(1,RMS, dist);
        end
        if verbose
            fprintf('Iteration %d RMS: %f\n', i, dist);
        end
        
        %check for convergence
        if i > 2
            if norm(RMS(end) - RMS(end - 1)) < tol && firs == 0
                toc;
                firs=1;
                fprintf('First Convergence after %d iterations for tol=%f\n', i, tol);
    
%                 break;
            end
        end
        
    end
    
    
    if i == max_iter
        fprintf('%d iterations run, no convergence for tol=%f!!\n', max_iter, tol);

    else
        fprintf('Convergence after %d iterations for tol=%f\n', i, tol);
    end
end

function [indices, distances] = match_pairs(A1, A2, kd_mdl)
%       kd_model is a KD-tree trained on the A2 points
        method = 'kd-tree';
        %method = 'pairwise';

        if strcmp(method, 'kd-tree') 
            [indices, distances] = knnsearch(kd_mdl, A1, 'K', 1);
        elseif strcmp(method, 'pairwise')
            [distances, indices] = min(pdist2(A1, A2), [], 2);
        end
           
end

function accepted_pairs = reject_pairs(p_distances)
    method = 'worst_percent';
    %method = 'threshold';
    %method = 'deviate';
    if strcmp(method, 'worst_percent') 
        [s_p_dists, s_p_distsI] = sort(p_distances);
        accepted_pairs = s_p_distsI(1:round(0.9*length(s_p_dists)));
    elseif strcmp(method, 'deviate') 
        m = mean(p_distances);
        s = std(p_distances);
        accepted_pairs = find(abs(p_distances - m) < 2.5 * s);
    elseif strcmp(method, 'threshold') 
    
    end
    
end


%% Root Mean Square
function dist = RootMeanSquare(A1_transf, psi_A1)
%     dist = sum(sum(((A1_transf-psi_A1).^2), 2));
dist = sum(norm(A1_transf-psi_A1) ^ 2);    
dist = sqrt(dist/size(A1_transf, 1));
    
    %
end

%% Sampling
function [sampled_points] = select_points(points, percentage, type, normals, frame_id)
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
    
    elseif (strcmp(type, 'informative'))
        [indexes, nan_indexes] = informative(percentage, normals, frame_id);
        points(nan_indexes,:) =[];
        sampled_points = points(indexes, :);
    end

end