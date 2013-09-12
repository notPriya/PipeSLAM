function descriptors = calculateDescriptors(I, keypoints)
    descriptors = extractFeatures(I, keypoints, 'Method', 'SURF', 'SURFSize', 128);
end