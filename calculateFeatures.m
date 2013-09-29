% Finds the important features in the image.
% Uses the built in Matlab SURF feature detector.
function keypoints = calculateFeatures(I)
    features = detectSURFFeatures(I, 'MetricThreshold', 500);
    keypoints = features.Location;
    
    % Throw out keypoints that are in the odometer bounding box.
    i = find((keypoints(:,1) < 147 | keypoints(:,1) > 286) | ...
             (keypoints(:,2) < 264 | keypoints(:,2) > 296));
    keypoints = keypoints(i, :);
end