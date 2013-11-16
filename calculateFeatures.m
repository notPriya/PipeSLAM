% Finds the important features in the image.
% Uses the built in Matlab SURF feature detector.
function [keypoints] = calculateFeatures(I, odom_rect, type)

if (nargin < 2)
    odom_rect = zeros(1, 4);
end
if (nargin < 3)
    type = 1;
end

margin = 25;

if (type == 1)
    
    % Detect the edges in the image.
    [~, t] = edge(I, 'canny');
    p = edge(I, 'canny', t + [.1 .3]);

    % Extract Keypoints from edges.
    [i, j] = find(p == 1);
    keypoints = [j i];
    
    % Throw out keypoints that are in the odometer bounding box.
    i = find((keypoints(:,1) < odom_rect(1) | keypoints(:,1) > odom_rect(2)) | ...
             (keypoints(:,2) < odom_rect(3) | keypoints(:,2) > odom_rect(4)));
    keypoints = keypoints(i, :);
    
    % Throw out keypoints that are too close to the edge.
    i = find((keypoints(:,1) > margin & keypoints(:,1) < (floor(size(I, 2)/margin)-1)*margin) & ...
             (keypoints(:,2) > margin & keypoints(:,2) < (floor(size(I, 1)/margin)-1)*margin));
    keypoints = keypoints(i, :);
    
    % If we dont have enough points, reduce the thresholding and try again.
    if (size(keypoints, 1) < 500)
        p = edge(I, 'canny', t + [0 .1]);
        
        % Extract Keypoints from edges.
        [i, j] = find(p == 1);
        keypoints = [j i];

        % Throw out keypoints that are in the odometer bounding box.
        i = find((keypoints(:,1) < odom_rect(1) | keypoints(:,1) > odom_rect(2)) | ...
                 (keypoints(:,2) < odom_rect(3) | keypoints(:,2) > odom_rect(4)));
        keypoints = keypoints(i, :);

        % Throw out keypoints that are too close to the edge.
        i = find((keypoints(:,1) > margin & keypoints(:,1) < (floor(size(I, 2)/margin)-1)*margin) & ...
                 (keypoints(:,2) > margin & keypoints(:,2) < (floor(size(I, 1)/margin)-1)*margin));
        keypoints = keypoints(i, :);
    end
    
    % Subsample points from the edges.
    n = length(keypoints);
    ind = randperm(length(keypoints), round(0.4*n));
    keypoints = keypoints(ind, :);

else
    
    

    features = detectSURFFeatures(I, 'MetricThreshold', 500);
    keypoints = features.Location;
    
    % Throw out keypoints that are in the odometer bounding box.
    i = find((keypoints(:,1) < odom_rect(1) | keypoints(:,1) > odom_rect(2)) | ...
             (keypoints(:,2) < odom_rect(3) | keypoints(:,2) > odom_rect(4)));
    keypoints = keypoints(i, :);
end

end