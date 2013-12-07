function [matches] = matchSIFT(D1, D2, t, type, A_ind)
if type == 1
    load('svm_linear_model.mat');
    
    n = size(D1, 2);
    max_val = zeros(n, 1);
    max_ind = zeros(n, 1);
    
    d2 = D2';
    for i = 1:n
        d = repmat(D1(:,i)', size(d2, 1), 1);
        test = double((d - d2).^2);
        testY = ones(size(test, 1), 1);
        
        [~, ~, p] = svmpredict(testY, test, mdl, '-b 1');
        
        [val, ind] = max(p(:,1));        
        max_val(i) = val;
        max_ind(i) = ind;
    end

    index = find(max_val > t);

    % Make sure we get at least 20 matches. Take the top 20 matches if
    % there are not enough.
    if (length(index) < 20)
        disp('Not enough matches');
    end
    
    matches = [index'; max_ind(index)'];
    
else
    if type == 2
        load('itml_results.mat');
        S = A{A_ind};
    else
        load('diag_covariance.mat');
    end
    
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
    min2_val = min(distances, [], 2);
    
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
end