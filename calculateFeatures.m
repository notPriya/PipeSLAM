% Finds the important features in the image.
% Uses the built in Matlab SURF feature detector.
function [keypoints] = calculateFeatures(I, type)
if (type == 1)
    
    % Detect the edges in the image.
    [~, t] = edge(I, 'canny');
    p = edge(I, 'canny', t + [.1 .3]);
    
    % Extract Keypoints from edges.
    [i, j] = find(p == 1);
    keypoints = [j i];
    
    % Throw out keypoints that are in the odometer bounding box.
    i = find((keypoints(:,1) < 385 | keypoints(:,1) > 581) | ...
             (keypoints(:,2) < 42 | keypoints(:,2) > 77));
    keypoints = keypoints(i, :);
    
    % Subsample points from the edges.
    n = length(keypoints);
    ind = randperm(length(keypoints), round(0.4*n));
    keypoints = keypoints(ind, :);

else
    
    

    features = detectSURFFeatures(I, 'MetricThreshold', 500);
    keypoints = features.Location;
    
    % Throw out keypoints that are in the odometer bounding box.
%     i = find((keypoints(:,1) < 276 | keypoints(:,1) > 422) | ...
%              (keypoints(:,2) < 23 | keypoints(:,2) > 59));
    i = find((keypoints(:,1) < 385 | keypoints(:,1) > 581) | ...
             (keypoints(:,2) < 42 | keypoints(:,2) > 77));
%     i = find((keypoints(:,1) < 166 | keypoints(:,1) > 287) | ...
%              (keypoints(:,2) < 269 | keypoints(:,2) > 295));
    keypoints = keypoints(i, :);
end

end