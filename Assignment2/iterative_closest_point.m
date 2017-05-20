%% iterative closest point
function [R,t, RMS,A1] = iterative_closest_point(A1, A2, sampling_method, sampling_percentage)
    %Run the ICP algorithm
    %A1 is the source point cloud
    %A2 is the target point cloud
    %normals1 are the normals of the baase point cloud
    %normals2 are the normals of the target point cloud
     
    tol = 1e-6;   
    max_iter = 100;
    verbose = false;
    rejection_method = 'worst_percent';
    weighting_method = 'constant';
    
    %% Initialize R = I, t = 0
    R = eye(3);
    t = zeros(1, 3);

    A1_init = A1;
    A2_init = A2;
    
    A1_transf = A1 * R + t;
    
    kd_mdl = KDTreeSearcher(A2);

    %% Find the closest points for each point in the base point set (A1) from the target point set (A2) using brute-force approach.
    tic;
    for i = 0 : max_iter        
        %match points
        [matches, distances] =  match_pairs(A1_transf, A2, kd_mdl);
        psi_A1 = A2(matches, :);
        
        %reject pairs; 
        if strcmp(rejection_method, 'none')
             A1_acc = A1;
             psi_A1_acc = psi_A1;
             accepted_pairs = [];
        else
            accepted_pairs = reject_pairs(rejection_method, distances);
            A1_acc = A1(accepted_pairs,:);
            psi_A1_acc = psi_A1(accepted_pairs,:);
            distances = distances(accepted_pairs);
        end
        
        %define weights
%         if (strcmp(weighting_method, 'max'))
%             w = weights(weighting_method, distances);
%         elseif (strcmp(weighting_method, 'normals'))
%             %Match the normals, keep the accepted normals and rotate the normals of the source
%             n2 = normals2(matches,:);
%             if isempty(accepted_pairs)
%                 n1 = normals1;
%             else
%                 n1 = normals1(accepted_pairs,:);
%                 n2 = n2(accepted_pairs,:);
%             end
%             n1 = n1*R;
%             w = weights(weighting_method, n1, n2);
%         elseif (strcmp(weighting_method, 'constant'))
%             w = weights(weighting_method, size(distances, 1));
%         end

        
        %Find rotation
%         [R,t] = svd_rot(A1_acc, psi_A1_acc, w);
        [R,t] = svd_rot(A1_acc, psi_A1_acc);
        
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
            if norm(RMS(end) - RMS(end - 1)) < tol 
                if verbose
                    toc; 
                    fprintf('First Convergence after %d iterations for tol=%f\n', i, tol);
                    break;
                end
            end
        end
        
    end
    
    if verbose
        if i == max_iter
            fprintf('%d iterations run, no convergence for tol=%f!!\n', max_iter, tol);
        else
            fprintf('Convergence after %d iterations for tol=%f\n', i, tol);
        end
    end
end

function [indices, distances] = match_pairs(A1, A2, kd_mdl)
%       Match the points in A1 with the closest points in A2 using a k-d
%       tree
%       kd_model is a KD-tree trained on the A2 points

        method = 'kd-tree';
        %method = 'pairwise';

        if strcmp(method, 'kd-tree') 
            [indices, distances] = knnsearch(kd_mdl, A1, 'K', 1);
        elseif strcmp(method, 'pairwise')
            [distances, indices] = min(pdist2(A1, A2), [], 2);
        else
            error(method, ' is not a matching pairs method');
        end
           
end


function [w] = weights(method, arg1, arg2)
%   Define the weights based on the weighting method
%   For constant weights, arg1 is the number of pairs
%   For max weights, arg1 are the distances between each pair
%   For normals weights, arg1 and arg2 are the normals of the base points
%   and the matching target points respectively
    if strcmp(method, 'constant') 
        N = arg1;
        w = ones(N, 1);
    elseif strcmp(method, 'max') 
        distances = arg1;
        max_dist = max(distances);
        w = 1 - distances/max_dist;
    elseif strcmp(method, 'normals')
        n1 = arg1;
        n2 = arg2;
        w = dot(n1,n2, 2);
        w(isnan(w)) = 0;
    else
        error(method, ' is not a weighting method');
    end

end

function accepted_pairs = reject_pairs(method, p_distances)
    %Return the indexes of the accepted pairs using one of the 2 available
    %methods
    if strcmp(method, 'worst_percent') 
        [s_p_dists, s_p_distsI] = sort(p_distances);
        accepted_pairs = s_p_distsI(1:round(0.9*length(s_p_dists)));
    elseif strcmp(method, 'deviate') 
        m = mean(p_distances);
        s = std(p_distances);
        accepted_pairs = find(abs(p_distances - m) < 2.5 * s); 
    else
        error(method, ' is not a rejecting pairs method');
    end
    
end


%% Root Mean Square
function dist = RootMeanSquare(A1_transf, psi_A1) 
    dist = sum(norm(A1_transf-psi_A1) ^ 2);    
    dist = sqrt(dist/size(A1_transf, 1)); 
end

%% Sampling
function [sampled_points] = select_points(points, percentage, method, normals)
%   Sample points using one of the 3 sampling methods (uniform, random,
%   normals)

    % initial number of points
    num_of_points = size(points, 1);

    % keep #num_of_sample points
    num_of_sample = ceil (num_of_points * percentage);

    if (strcmp(method, 'all'))
        sampled_points = points;

    elseif (strcmp(method, 'uniform'))    
        indexes = int64(linspace(1, num_of_points, num_of_sample));
        sampled_points = points(indexes, :);

    elseif (strcmp(method, 'random'))
        indexes = randsample(num_of_points, num_of_sample);
        sampled_points = points(indexes, :);
    
    elseif (strcmp(method, 'normal'))
        [indexes, nan_indexes] = normal_space_sampling(percentage, normals, 4);
        points(nan_indexes,:) = [];
        sampled_points = points(indexes, :);
    else
        error(method, ' is not a sampling method');
    end
           

end


function [indexes, nan_ind] = normal_space_sampling(percentage, normals, num_of_buckets)
   %Sample uniformly on the normal-space, keeping a certain percentage of
   %the normals and num_of_buckets

    % remove NaN normals
    nan_ind = find(isnan(normals(:,1)));
    normals(nan_ind,:)=[];

    %Convert to angular space
    angles_x = [];
    angles_y = [];
    for i=1 : size(normals,1)
        p = normals(i,:);
        % angle in rand w.r.t axis x
        x = atan2(sqrt(p(2)^2 + p(3)^2), p(1));
        % angle in rand w.r.t axis y
        y = atan2(sqrt(p(1)^2 + p(3)^2), p(2));

        angles_x = [angles_x ; x];
        angles_y = [angles_y ; y];
    end

    % create buckets w.r.t angles on x and y
    [bucket_x, edges_x] = discretize(angles_x, num_of_buckets);
    [bucket_y, edges_y] = discretize(angles_y, num_of_buckets);

    bucket = num_of_buckets * (bucket_x - 1) + bucket_y;

    %create invert index so that for each bucket we have the indexes of the points in that
    %bucket
    max_id = num_of_buckets * num_of_buckets;
    inv_indexes = cell(1, max_id);

    for i=1 : max_id 
        inv_indexes{i} =  find(bucket == i);
    end


    %% uniformly sample from buckets

    % initial number of points
    num_of_points = size(normals, 1);
    % keep #num_of_sample points
    num_of_sample = ceil (num_of_points * percentage);
    % keep #num_of_sample_per_bucket from each bucket
    num_of_sample_per_bucket = ceil(num_of_sample / max_id);

    %Fill in the indexes uniformly by choosing from each bucket (randomly) the same
    %ammount of points
    indexes = [];
    for i=1 : max_id
        %If there are not enough points in the bucket, then take all of them 
        if num_of_sample_per_bucket < size(inv_indexes{i}, 1)
            indexes_2 = randsample(inv_indexes{i}, num_of_sample_per_bucket);
            indexes = [indexes; indexes_2];
        else
            indexes = [indexes; inv_indexes{i}];
        end
    end

    %% visualize buckets
    % scatter3(normals(:,1),normals(:,2),normals(:,3),ones(size(normals,1),1),bucket)

end