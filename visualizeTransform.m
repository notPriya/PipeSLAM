% Computes the transform between one frame and the next and visualizes the
% results.
function visualizeTransform(I1, I2, odom_rect)
    % Calculate the transform between I1 and I2.
    [H, E] = calculateTransform(I1, I2, odom_rect);

    % Visualize the epipolar lines.
    epipolarMatchGUI(I1, I2, E);
end