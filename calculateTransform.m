function H = calculateTransform(I1, I2)
    % Extract features and descriptors for the first image.
    im1 = preprocessImage(I1);
    k1 = calculateFeatures(im1);
    d1 = calculateDescriptors(im1, k1);
    
    % Extract features and descriptors for the second image.
    im2 = preprocessImage(I2);
    k2 = calculateFeatures(im2);
    d2 = calculateDescriptors(im2, k2);

    % Compute matches
    matches = matchSIFT(d1', d2', 0.8);

    % Run RANSAC to get the best model.
    H = ransac(k1', k2', matches);
end