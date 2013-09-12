function [matches] = matchSIFT(D1, D2, t)
    % Compute distances from everything in D1 to everything in D2
    distances = pdist2(D1', D2');
    
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
    
    % Compute result
    matches = [index'; min_ind(index)'];
end