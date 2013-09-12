% Finds the important features in the image.
% Uses the built in Matlab SURF feature detector.
function keypoints = calculateFeatures(I)
    features = detectSURFFeatures(I, 'MetricThreshold', 500);
    keypoints = features.Location;
end