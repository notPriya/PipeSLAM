function [matches] = matchSIFT(D1, D2, t)
    load('diag_covariance.mat');
    
    % Compute distances from everything in D1 to everything in D2
    distances = pdist2(D1', D2', 'mahalanobis', S);
    
    % Compute the nearest neighbor
    [min_val, min_ind] = min(distances, [], 2);
    
    % Mask out the nearest neighbor
    i = 1:numel(min_ind);
    j = min_ind;
    index = sub2ind(size(distances), i', j);
    distances(index) = NaN;
    
    % Find second nearnest neighbor.
    [min2_val, min2_ind] = min(distances, [], 2);
    
    % Compute thresholded value.
    threshold = min_val ./ min2_val;
    
    % Find accepted matches that are below threshold.
    index = find(threshold < t);
    
    % Make sure we get at least 20 matches. Take the top 20 matches if
    % there are not enough.
    if (length(index) < 20)
        [~, index] = sort(threshold, 1, 'ascend');
        index = index(1:20);
    end
    
    % Compute result
    matches = [index'; min_ind(index)'];
end