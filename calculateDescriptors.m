function descriptors = calculateDescriptors(I, keypoints, original)
    % Calculate the normal SURF descriptors
    surf = extractFeatures(I, keypoints, 'Method', 'SURF', 'SURFSize', 128);
    
    % Calculate the HOG descriptors over the entire image.
    hog = features(original, 9);
    % Match keypoints to HOG descriptors.
    ind = floor(keypoints./9);
    hogs = zeros(length(keypoints), 9);
    for i=1:length(ind)
        hogs(i, :) = hog(ind(1), ind(2), 1:9);
    end
    
    % Add the normalized location to the descriptors.
    location(:, 1) = keypoints(:, 1)/size(I, 2);
    location(:, 2) = keypoints(:, 2)/size(I, 1);
    
    descriptors = [surf hogs location];
end