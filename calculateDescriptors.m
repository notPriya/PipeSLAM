function descriptors = calculateDescriptors(I, keypoints)
    % Calculate the normal SURF descriptors
    descriptors = extractFeatures(I, keypoints, 'Method', 'SURF', 'SURFSize', 128);
    
    % Add the normalized location to the descriptors.
    location(:, 1) = keypoints(:, 1)/size(I, 2);
    location(:, 2) = keypoints(:, 2)/size(I, 1);
    
    descriptors = [descriptors location];
end