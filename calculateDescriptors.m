function descriptors = calculateDescriptors(I, keypoints, original)
    % Calculate the normal SURF descriptors
    surf = extractFeatures(I, keypoints, 'Method', 'SURF', 'SURFSize', 128);

    % Calculate HOG features over a couple different bin sizes
    original = double(original);
    hogs = [getHOGFeatures(original, keypoints, 13) ...
            getHOGFeatures(original, keypoints, 20) ...
            getHOGFeatures(original, keypoints, 25)];
    
    % Add the normalized location to the descriptors.
    location(:, 1) = keypoints(:, 1)/size(I, 2);
    location(:, 2) = keypoints(:, 2)/size(I, 1);
    
    descriptors = [surf hogs location];
end

function hogs = getHOGFeatures(I, keypoints, bin_size)
    % Compute HOG features over the entire image.
    hog = features(I, bin_size);
    
    % Match keypoints to HOG cells.
    ind = floor(keypoints./bin_size);
    ind(ind<1) = 1;
    ind(ind(:,1) > size(hog, 2), 1) = size(hog, 2);
    ind(ind(:,2) > size(hog, 1), 2) = size(hog, 1);
    
    % Match keypoints to HOG descriptors.
    hogs = zeros(length(keypoints), 9);
    for i=1:length(ind)
        hogs(i, :) = hog(ind(i, 2), ind(i, 1), 1:9);
    end
end