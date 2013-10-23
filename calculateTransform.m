function [H, E] = calculateTransform(I1, I2)
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

    % Extract the matched points.
    p1 = k1(matches(1, :), 1:2);
    p2 = k2(matches(2, :), 1:2);
    
    % Normalize by image coordinates to get better estimates.
    p1 = p1./max(size(I1));
    p2 = p2./max(size(I1))
    
    % Run RANSAC to get the best model.
%     H = ransac(k1', k2', matches);
    [H, E] = computeEssentialMatrix(p1, p2);
end