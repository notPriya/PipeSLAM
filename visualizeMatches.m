%Runs SIFT matching and visualizes the results.
function visualizeMatches(I1, I2, t)
    % Extract features and descriptors for the first image.
    im1 = preprocessImage(I1);
    k1 = calculateFeatures(im1);
    d1 = calculateDescriptors(im1, k1);
    
    % Extract features and descriptors for the second image.
    im2 = preprocessImage(I2);
    k2 = calculateFeatures(im2);
    d2 = calculateDescriptors(im2, k2);

    %compute matches
    matches = matchSIFT(d1', d2', 0.8);

    %concatenate the images
    [h1,w1] = size(im1);
    [h2,w2] = size(im2);
    catImage = uint8(zeros([max(h1,h2),w1+w2]));
    catImage(1:h1,1:w1) = im1;
    catImage(1:h2,w1+1:w1+w2) = im2;
    imshow(catImage); hold on;
    
    %collect the correspondences more conveniently
    corresp1 = k1(matches(1,:), 1:2)';
    corresp2 = k2(matches(2,:), 1:2)';
    for i=1:min(size(matches,2), t)
        plot(   [corresp1(1,i),corresp2(1,i)+w1],...
                [corresp1(2,i),corresp2(2,i)]);
    end
end

