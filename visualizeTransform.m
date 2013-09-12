% Computes the transform between one frame and the next and visualizes the
% results.
function visualizeTransform(I1, I2)
    % Calculate the transform between I1 and I2.
    H = calculateTransform(I1, I2);

    close all;
    % Display the first image
    imshow(I1);
    figure;
    
    % Warp the second image in the frame of the first image.
    I2to1 = warpH(I2, double(H), size(I2), 3);
    imshow(I2to1);
end