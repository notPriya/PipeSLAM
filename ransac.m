function [bestH, inliers] = ransac(keypoints1, keypoints2, matches)
    % Number of iterations. Determine later.
    n = 7;  % We need at least 4 correspondences to determine H.
    w = .3; % Percentage of inliers.
    T = ceil(log(1-0.9)/log(1-w^n));
    
    % Save off matches so it is easier to use.
    A = keypoints1(1:2, matches(1, :));
    B = keypoints2(1:2, matches(2, :));
    
    % Initialize loop variables
    num_inliers = [];
    inliers_set = cell(T, 1);
    H_val = cell(T, 1);
    
    % RANSAC
    for i = 1:T
        % Pick n random correspondences from matches to use
        index = randperm(size(matches, 2));
        index = index(1:n);
        
        % Ideally we would check to make sure that we dont
        % get duplicate matches, but this works fine without
        % the checking, so it is not implemented.
                
        % Get the n matches and add to model set
        points = matches(:, index);
        
        % Compute H using model set
        H = computeH_norm(keypoints1(1:2, points(1, :)), ...
                          keypoints2(1:2, points(2, :)));
                      
        % Find number of inliers_set
        % Compute Transform
        HB = H*[B; ones(1, size(B, 2))];
        % Normalize result
        HB(1, :) = HB(1, :) ./ HB(3, :);
        HB(2, :) = HB(2, :) ./ HB(3, :);
        HB(3, :) = HB(3, :) ./ HB(3, :);
        % Find distance^2
        error = sum((A - HB(1:2, :)).^2);
        % Find inliers_set. Index is indexes into matches.
        index = find(error < 5);
        
        inliers_set{i} = index;    
        H_val{i} = H;
        num_inliers = [num_inliers; numel(index)];
    end

    % Find maximum number of inliers that the best model has found.
    [max_val, index] = max(num_inliers);
    indices = find(num_inliers == max_val);
    
    % Check if there is more than one candidate model.
    if length(indices) > 1
        % Pick the model with the largest trace, closest to identity.
        trace = zeros(length(indices));
        for i=1:length(indices)
            H = H_val{indices(i)};
            trace(i) = trace(H(1:3, 1:3));
        end
        [~, ind] = max(trace);
        index = indices(ind);
    end
    
    
    % Find the corresponding H and inliers_set
    bestH = H_val{index};
    inliers = inliers_set{index};
        
end